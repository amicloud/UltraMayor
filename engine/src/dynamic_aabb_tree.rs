use bevy_ecs::entity::Entity;
use glam::Vec3;

use crate::mesh::AABB;

pub type NodeId = usize;

#[derive(Debug, Default)]
struct Node {
    aabb: AABB,
    parent: Option<NodeId>,
    left: Option<NodeId>,
    right: Option<NodeId>,
    height: i32,
    entity: Option<Entity>, // Some => leaf
}

#[derive(Debug, Default)]
pub struct DynamicAabbTree {
    nodes: Vec<Node>,
    root: Option<NodeId>,
    free_list: Vec<NodeId>,
}

impl DynamicAabbTree {
    pub fn update(&mut self, node_id: NodeId, new_aabb: AABB) {
        let fat = Self::expand_aabb(new_aabb, 0.1);

        if self.nodes[node_id].aabb.contains(&fat) {
            return; // still inside fat AABB, no reinsertion needed
        }

        self.remove(node_id);
        self.insert_leaf(node_id, fat);
    }

    fn is_leaf(&self, id: NodeId) -> bool {
        self.nodes[id].left.is_none()
    }

    fn expand_aabb(aabb: AABB, margin: f32) -> AABB {
        AABB {
            min: aabb.min - Vec3::splat(margin),
            max: aabb.max + Vec3::splat(margin),
        }
    }

    pub fn allocate_leaf(&mut self, entity: Entity, aabb: AABB) -> NodeId {
        let leaf = self.allocate_node();

        let fat = Self::expand_aabb(aabb, 0.1);

        self.nodes[leaf].aabb = fat;
        self.nodes[leaf].entity = Some(entity);
        self.nodes[leaf].height = 0;
        self.nodes[leaf].left = None;
        self.nodes[leaf].right = None;

        self.insert_leaf(leaf, fat);

        leaf
    }

    pub fn remove(&mut self, leaf: NodeId) {
        if self.root == Some(leaf) {
            self.root = None;
            return;
        }

        let parent = self.nodes[leaf].parent.unwrap();
        let grand_parent = self.nodes[parent].parent;

        // Determine sibling
        let sibling = if self.nodes[parent].left == Some(leaf) {
            self.nodes[parent].right.unwrap()
        } else {
            self.nodes[parent].left.unwrap()
        };

        if let Some(gp) = grand_parent {
            // Replace parent with sibling in grandparent
            if self.nodes[gp].left == Some(parent) {
                self.nodes[gp].left = Some(sibling);
            } else {
                self.nodes[gp].right = Some(sibling);
            }

            self.nodes[sibling].parent = Some(gp);

            // Fix tree upward
            self.fix_upwards(gp);
        } else {
            // Parent was root
            self.root = Some(sibling);
            self.nodes[sibling].parent = None;
        }

        // Clear parent link on removed leaf
        self.nodes[leaf].parent = None;

        // Optional: recycle parent node via free list
    }

    pub fn insert_leaf(&mut self, leaf: NodeId, aabb: AABB) {
        self.nodes[leaf].aabb = aabb;
        self.nodes[leaf].left = None;
        self.nodes[leaf].right = None;
        self.nodes[leaf].height = 0;

        if self.root.is_none() {
            self.root = Some(leaf);
            self.nodes[leaf].parent = None;
            return;
        }

        // 1. Find best sibling
        let mut index = self.root.unwrap();

        while !self.is_leaf(index) {
            let left = self.nodes[index].left.unwrap();
            let right = self.nodes[index].right.unwrap();

            let area = self.nodes[index].aabb.area();

            let combined = self.nodes[index].aabb.union(&aabb);
            let combined_area = combined.area();

            // Cost of creating new parent here
            let cost = 2.0 * combined_area;

            // Minimum cost of pushing down
            let inheritance_cost = 2.0 * (combined_area - area);

            let cost_left = {
                let union = self.nodes[left].aabb.union(&aabb);
                if self.is_leaf(left) {
                    union.area() + inheritance_cost
                } else {
                    union.area() - self.nodes[left].aabb.area() + inheritance_cost
                }
            };

            let cost_right = {
                let union = self.nodes[right].aabb.union(&aabb);
                if self.is_leaf(right) {
                    union.area() + inheritance_cost
                } else {
                    union.area() - self.nodes[right].aabb.area() + inheritance_cost
                }
            };

            if cost < cost_left && cost < cost_right {
                break; // create new parent here instead of descending
            } else if cost_left < cost_right {
                // Descend
                index = left;
            } else {
                index = right;
            }
        }

        let sibling = index;
        let old_parent = self.nodes[sibling].parent;

        // 2. Create new parent
        let new_parent = self.allocate_node();

        self.nodes[new_parent].parent = old_parent;
        self.nodes[new_parent].aabb = self.nodes[sibling].aabb.union(&aabb);
        self.nodes[new_parent].height = self.nodes[sibling].height + 1;
        self.nodes[new_parent].left = Some(sibling);
        self.nodes[new_parent].right = Some(leaf);
        self.nodes[new_parent].entity = None;

        self.nodes[sibling].parent = Some(new_parent);
        self.nodes[leaf].parent = Some(new_parent);

        if let Some(parent) = old_parent {
            if self.nodes[parent].left == Some(sibling) {
                self.nodes[parent].left = Some(new_parent);
            } else {
                self.nodes[parent].right = Some(new_parent);
            }
        } else {
            self.root = Some(new_parent);
        }

        // 3. Walk upward fixing AABBs/heights
        self.fix_upwards(new_parent);
    }

    fn update_node(&mut self, node: NodeId) {
        let left = self.nodes[node].left.unwrap();
        let right = self.nodes[node].right.unwrap();

        self.nodes[node].height = 1 + self.nodes[left].height.max(self.nodes[right].height);
        self.nodes[node].aabb = self.nodes[left].aabb.union(&self.nodes[right].aabb);
    }

    fn fix_upwards(&mut self, mut index: NodeId) {
        loop {
            // Update height/AABB first
            self.update_node(index);

            // Check balance
            let left = self.nodes[index].left.unwrap();
            let right = self.nodes[index].right.unwrap();
            let balance = self.nodes[left].height as isize - self.nodes[right].height as isize;

            // Perform rotation if needed, get new root of this subtree
            index = if balance > 1 {
                self.rotate_right(index)
            } else if balance < -1 {
                self.rotate_left(index)
            } else {
                index
            };

            // Move up to parent of the current subtree root
            if let Some(parent) = self.nodes[index].parent {
                index = parent;
            } else {
                break;
            }
        }
    }

    fn rotate_right(&mut self, node: NodeId) -> NodeId {
        let left = self.nodes[node].left.unwrap();
        let left_right = self.nodes[left].right;

        // Left becomes new parent
        self.nodes[left].parent = self.nodes[node].parent;
        self.nodes[node].parent = Some(left);

        // Update children
        self.nodes[left].right = Some(node);
        self.nodes[node].left = left_right;

        if let Some(lr) = left_right {
            self.nodes[lr].parent = Some(node);
        }

        // Update parent pointer
        if let Some(parent) = self.nodes[left].parent {
            if self.nodes[parent].left == Some(node) {
                self.nodes[parent].left = Some(left);
            } else {
                self.nodes[parent].right = Some(left);
            }
        } else {
            self.root = Some(left);
        }

        // Recompute heights and AABBs
        self.update_node(node);
        self.update_node(left);
        left
    }

    fn rotate_left(&mut self, node: NodeId) -> NodeId {
        let right = self.nodes[node].right.unwrap();
        let right_left = self.nodes[right].left;

        // Right becomes new parent
        self.nodes[right].parent = self.nodes[node].parent;
        self.nodes[node].parent = Some(right);

        // Update children
        self.nodes[right].left = Some(node);
        self.nodes[node].right = right_left;

        if let Some(rl) = right_left {
            self.nodes[rl].parent = Some(node);
        }

        // Update parent pointer
        if let Some(parent) = self.nodes[right].parent {
            if self.nodes[parent].left == Some(node) {
                self.nodes[parent].left = Some(right);
            } else {
                self.nodes[parent].right = Some(right);
            }
        } else {
            self.root = Some(right);
        }

        // Recompute heights and AABBs
        self.update_node(node);
        self.update_node(right);
        right
    }

    fn allocate_node(&mut self) -> NodeId {
        if let Some(id) = self.free_list.pop() {
            id
        } else {
            let id = self.nodes.len();
            self.nodes.push(Node {
                aabb: AABB::default(),
                parent: None,
                left: None,
                right: None,
                height: 0,
                entity: None,
            });
            id
        }
    }
    pub fn query<F>(&self, aabb: AABB, mut callback: F)
    where
        F: FnMut(Entity),
    {
        if let Some(root) = self.root {
            self.query_node(root, &aabb, &mut callback);
        }
    }

    fn query_node<F>(&self, node_id: NodeId, aabb: &AABB, callback: &mut F)
    where
        F: FnMut(Entity),
    {
        let node = &self.nodes[node_id];

        if !node.aabb.intersects(aabb) {
            return;
        }

        if let Some(entity) = node.entity {
            callback(entity);
        } else {
            self.query_node(node.left.unwrap(), aabb, callback);
            self.query_node(node.right.unwrap(), aabb, callback);
        }
    }
}
