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

        // Recycle parent node via free list
        self.recycle_node(parent);
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

    fn recycle_node(&mut self, id: NodeId) {
        self.nodes[id].aabb = AABB::default();
        self.nodes[id].parent = None;
        self.nodes[id].left = None;
        self.nodes[id].right = None;
        self.nodes[id].height = 0;
        self.nodes[id].entity = None;
        self.free_list.push(id);
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

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::seq::SliceRandom;
    use rand::{Rng, SeedableRng};
    use std::collections::HashSet;

    fn make_aabb(center: Vec3, half_extent: f32) -> AABB {
        AABB {
            min: center - Vec3::splat(half_extent),
            max: center + Vec3::splat(half_extent),
        }
    }

    fn assert_aabb_eq(left: AABB, right: AABB, message: &str) {
        assert_eq!(left.min, right.min, "{message} (min)");
        assert_eq!(left.max, right.max, "{message} (max)");
    }

    fn validate_node(
        tree: &DynamicAabbTree,
        node_id: NodeId,
        expected_parent: Option<NodeId>,
        visited: &mut HashSet<NodeId>,
        check_balance: bool,
        leaves: &mut Vec<NodeId>,
    ) -> (i32, AABB) {
        assert!(visited.insert(node_id), "cycle detected at node {node_id}");

        let node = &tree.nodes[node_id];
        assert_eq!(node.parent, expected_parent, "parent pointer mismatch");

        if tree.is_leaf(node_id) {
            assert!(node.entity.is_some(), "leaf should have entity");
            assert!(node.left.is_none(), "leaf left should be None");
            assert!(node.right.is_none(), "leaf right should be None");
            assert_eq!(node.height, 0, "leaf height should be 0");
            leaves.push(node_id);
            return (0, node.aabb);
        }

        assert!(
            node.entity.is_none(),
            "internal node should not have entity"
        );
        let left = node.left.expect("internal node left missing");
        let right = node.right.expect("internal node right missing");
        assert_ne!(left, right, "internal node children must differ");

        let (left_height, left_aabb) =
            validate_node(tree, left, Some(node_id), visited, check_balance, leaves);
        let (right_height, right_aabb) =
            validate_node(tree, right, Some(node_id), visited, check_balance, leaves);

        let expected_height = 1 + left_height.max(right_height);
        let expected_aabb = left_aabb.union(&right_aabb);

        assert_eq!(node.height, expected_height, "height mismatch");
        assert_aabb_eq(node.aabb, expected_aabb, "AABB mismatch");

        if check_balance {
            let balance = left_height - right_height;
            assert!(
                (-2..=2).contains(&balance),
                "balance factor out of range: {balance}"
            );
        }

        (expected_height, node.aabb)
    }

    fn assert_tree_invariants(tree: &DynamicAabbTree, check_balance: bool) -> (i32, Vec<NodeId>) {
        if let Some(root) = tree.root {
            assert_eq!(tree.nodes[root].parent, None, "root parent must be None");
            let mut visited = HashSet::new();
            let mut leaves = Vec::new();
            let (height, _) =
                validate_node(tree, root, None, &mut visited, check_balance, &mut leaves);
            (height, leaves)
        } else {
            (-1, Vec::new())
        }
    }

    fn add_leaf_node(tree: &mut DynamicAabbTree, entity: Entity, aabb: AABB) -> NodeId {
        let id = tree.allocate_node();
        tree.nodes[id].aabb = aabb;
        tree.nodes[id].entity = Some(entity);
        tree.nodes[id].height = 0;
        tree.nodes[id].left = None;
        tree.nodes[id].right = None;
        tree.nodes[id].parent = None;
        id
    }

    fn add_internal_node(tree: &mut DynamicAabbTree, left: NodeId, right: NodeId) -> NodeId {
        let id = tree.allocate_node();
        tree.nodes[id].entity = None;
        tree.nodes[id].left = Some(left);
        tree.nodes[id].right = Some(right);
        tree.nodes[id].parent = None;
        tree.nodes[left].parent = Some(id);
        tree.nodes[right].parent = Some(id);
        tree.nodes[id].height = 1 + tree.nodes[left].height.max(tree.nodes[right].height);
        tree.nodes[id].aabb = tree.nodes[left].aabb.union(&tree.nodes[right].aabb);
        id
    }

    #[test]
    fn leaf_insertion_test() {
        let mut tree = DynamicAabbTree::default();

        let leaf1 = tree.allocate_leaf(Entity::from_bits(1), make_aabb(Vec3::ZERO, 1.0));
        assert_eq!(tree.root, Some(leaf1));
        assert_tree_invariants(&tree, false);

        let leaf2 = tree.allocate_leaf(
            Entity::from_bits(2),
            make_aabb(Vec3::new(5.0, 0.0, 0.0), 1.0),
        );
        let root = tree.root.expect("root should exist after two inserts");
        assert_ne!(root, leaf1);
        assert_ne!(root, leaf2);
        let root_node = &tree.nodes[root];
        assert!(root_node.entity.is_none());
        assert!(root_node.left.is_some());
        assert!(root_node.right.is_some());
        assert_tree_invariants(&tree, false);

        let _leaf3 = tree.allocate_leaf(
            Entity::from_bits(3),
            make_aabb(Vec3::new(-5.0, 0.0, 0.0), 1.0),
        );
        assert_tree_invariants(&tree, false);
    }

    #[test]
    fn update_leaf_in_place_and_reinsert_test() {
        let mut tree = DynamicAabbTree::default();
        let base = make_aabb(Vec3::ZERO, 1.0);
        let leaf = tree.allocate_leaf(Entity::from_bits(100), base);

        let original_parent = tree.nodes[leaf].parent;
        let original_root = tree.root;
        let original_fat = tree.nodes[leaf].aabb;

        let inside_move = make_aabb(Vec3::new(0.05, 0.0, 0.0), 0.9);
        tree.update(leaf, inside_move);

        assert_eq!(tree.root, original_root);
        assert_eq!(tree.nodes[leaf].parent, original_parent);
        assert_aabb_eq(
            tree.nodes[leaf].aabb,
            original_fat,
            "AABB should remain unchanged",
        );
        assert_tree_invariants(&tree, false);

        let outside_move = make_aabb(Vec3::new(10.0, 0.0, 0.0), 1.0);
        tree.update(leaf, outside_move);
        let expected_fat = DynamicAabbTree::expand_aabb(outside_move, 0.1);
        assert_aabb_eq(
            tree.nodes[leaf].aabb,
            expected_fat,
            "AABB should update after reinsert",
        );
        assert_tree_invariants(&tree, true);
    }

    #[test]
    fn free_list_reuse_test() {
        let mut tree = DynamicAabbTree::default();
        let leaf1 = tree.allocate_leaf(Entity::from_bits(1), make_aabb(Vec3::ZERO, 1.0));
        let leaf2 = tree.allocate_leaf(
            Entity::from_bits(2),
            make_aabb(Vec3::new(5.0, 0.0, 0.0), 1.0),
        );

        let internal_root = tree.root.expect("root should exist");
        assert!(tree.nodes[internal_root].entity.is_none());

        tree.remove(leaf1);
        assert!(tree.free_list.contains(&internal_root));

        let leaf3 = tree.allocate_leaf(
            Entity::from_bits(3),
            make_aabb(Vec3::new(-5.0, 0.0, 0.0), 1.0),
        );
        assert_eq!(leaf3, internal_root, "expected reused node id");

        tree.remove(leaf2);
        assert_tree_invariants(&tree, false);
    }

    #[test]
    fn tree_rotation_balancing_test() {
        let mut tree = DynamicAabbTree::default();
        let mut leaf_count = 0usize;

        for i in 0..25usize {
            let x = i as f32 * 2.0;
            tree.allocate_leaf(
                Entity::from_bits((i + 1) as u64),
                make_aabb(Vec3::new(x, 0.0, 0.0), 0.5),
            );
            leaf_count += 1;

            let (height, leaves) = assert_tree_invariants(&tree, true);
            assert_eq!(leaves.len(), leaf_count);

            let max_height = (leaf_count as f32).log2().ceil() as i32 + 1;
            assert!(
                height <= max_height,
                "height {height} exceeds rough bound {max_height}"
            );
        }
    }

    #[test]
    fn rotation_edge_case_nested_subtree_test() {
        let mut tree = DynamicAabbTree::default();

        let leaf0 = add_leaf_node(&mut tree, Entity::from_bits(1), make_aabb(Vec3::ZERO, 1.0));
        let leaf1 = add_leaf_node(
            &mut tree,
            Entity::from_bits(2),
            make_aabb(Vec3::new(2.0, 0.0, 0.0), 1.0),
        );
        let leaf2 = add_leaf_node(
            &mut tree,
            Entity::from_bits(3),
            make_aabb(Vec3::new(4.0, 0.0, 0.0), 1.0),
        );
        let leaf3 = add_leaf_node(
            &mut tree,
            Entity::from_bits(4),
            make_aabb(Vec3::new(8.0, 0.0, 0.0), 1.0),
        );

        let left_left = add_internal_node(&mut tree, leaf0, leaf1);
        let left = add_internal_node(&mut tree, left_left, leaf2);
        let root = add_internal_node(&mut tree, left, leaf3);
        tree.root = Some(root);

        assert_tree_invariants(&tree, false);

        tree.fix_upwards(root);
        assert_tree_invariants(&tree, true);

        let new_root = tree.root.expect("root should exist after rotation");
        assert_eq!(new_root, left);
        assert_eq!(tree.nodes[new_root].parent, None);
    }

    #[test]
    fn leaf_removal_test() {
        // Root leaf removal
        let mut tree = DynamicAabbTree::default();
        let leaf = tree.allocate_leaf(Entity::from_bits(10), make_aabb(Vec3::ZERO, 1.0));
        tree.remove(leaf);
        assert!(tree.root.is_none());
        assert_eq!(tree.nodes[leaf].parent, None);

        // Remove middle leaf in a larger tree
        let mut tree = DynamicAabbTree::default();
        let leaf1 = tree.allocate_leaf(Entity::from_bits(1), make_aabb(Vec3::ZERO, 1.0));
        let leaf2 = tree.allocate_leaf(
            Entity::from_bits(2),
            make_aabb(Vec3::new(5.0, 0.0, 0.0), 1.0),
        );
        let leaf3 = tree.allocate_leaf(
            Entity::from_bits(3),
            make_aabb(Vec3::new(-5.0, 0.0, 0.0), 1.0),
        );

        let parent = tree.nodes[leaf2].parent.expect("leaf2 parent missing");
        let sibling = if tree.nodes[parent].left == Some(leaf2) {
            tree.nodes[parent].right.expect("sibling missing")
        } else {
            tree.nodes[parent].left.expect("sibling missing")
        };
        let grand_parent = tree.nodes[parent].parent;

        tree.remove(leaf2);
        assert_eq!(tree.nodes[leaf2].parent, None);
        if let Some(gp) = grand_parent {
            assert_eq!(tree.nodes[sibling].parent, Some(gp));
        } else {
            assert_eq!(tree.root, Some(sibling));
            assert_eq!(tree.nodes[sibling].parent, None);
        }
        assert_tree_invariants(&tree, false);

        // Remove another leaf and validate structure
        tree.remove(leaf3);
        assert_eq!(tree.nodes[leaf3].parent, None);
        assert_tree_invariants(&tree, false);
        assert!(tree.root.is_some() || tree.nodes[leaf1].parent.is_none());
    }

    #[test]
    fn degenerate_aabb_test() {
        let mut tree = DynamicAabbTree::default();
        let zero = AABB {
            min: Vec3::new(1.0, 1.0, 1.0),
            max: Vec3::new(1.0, 1.0, 1.0),
        };
        let overlapping = make_aabb(Vec3::new(1.0, 1.0, 1.0), 0.5);
        let touching = AABB {
            min: Vec3::new(2.0, 1.0, 1.0),
            max: Vec3::new(2.0, 1.0, 1.0),
        };

        let e1 = Entity::from_bits(10);
        let e2 = Entity::from_bits(11);
        let e3 = Entity::from_bits(12);

        tree.allocate_leaf(e1, zero);
        tree.allocate_leaf(e2, overlapping);
        tree.allocate_leaf(e3, touching);

        let query = AABB {
            min: Vec3::new(1.0, 1.0, 1.0),
            max: Vec3::new(2.0, 1.0, 1.0),
        };

        let mut found = HashSet::new();
        tree.query(query, |entity| {
            found.insert(entity);
        });

        assert!(found.contains(&e1));
        assert!(found.contains(&e2));
        assert!(found.contains(&e3));
    }

    #[test]
    fn query_test() {
        let mut tree = DynamicAabbTree::default();
        let mut leaves = Vec::new();

        let aabb1 = make_aabb(Vec3::new(0.0, 0.0, 0.0), 1.0);
        let aabb2 = make_aabb(Vec3::new(3.0, 0.0, 0.0), 1.0);
        let aabb3 = make_aabb(Vec3::new(10.0, 0.0, 0.0), 1.0);
        let aabb4 = make_aabb(Vec3::new(-4.0, 0.0, 0.0), 0.5);

        let e1 = Entity::from_bits(1);
        let e2 = Entity::from_bits(2);
        let e3 = Entity::from_bits(3);
        let e4 = Entity::from_bits(4);

        let l1 = tree.allocate_leaf(e1, aabb1);
        let l2 = tree.allocate_leaf(e2, aabb2);
        let l3 = tree.allocate_leaf(e3, aabb3);
        let l4 = tree.allocate_leaf(e4, aabb4);

        leaves.push((l1, e1, aabb1));
        leaves.push((l2, e2, aabb2));
        leaves.push((l3, e3, aabb3));
        leaves.push((l4, e4, aabb4));

        let query = make_aabb(Vec3::new(2.0, 0.0, 0.0), 2.0);

        let mut found = HashSet::new();
        tree.query(query, |entity| {
            found.insert(entity);
        });

        let mut expected = HashSet::new();
        for (_, entity, aabb) in &leaves {
            let fat = DynamicAabbTree::expand_aabb(*aabb, 0.1);
            if fat.intersects(&query) {
                expected.insert(*entity);
            }
        }

        assert_eq!(found, expected);
    }

    #[test]
    fn query_edge_cases_test() {
        let mut tree = DynamicAabbTree::default();
        let mut found = Vec::new();
        tree.query(make_aabb(Vec3::ZERO, 1.0), |entity| {
            found.push(entity);
        });
        assert!(found.is_empty());

        let e1 = Entity::from_bits(100);
        let e2 = Entity::from_bits(101);
        let l1 = tree.allocate_leaf(e1, make_aabb(Vec3::new(-1.0, 0.0, 0.0), 1.0));
        let l2 = tree.allocate_leaf(e2, make_aabb(Vec3::new(2.0, 0.0, 0.0), 1.0));
        let world_query = make_aabb(Vec3::new(0.5, 0.0, 0.0), 10.0);

        let mut found = HashSet::new();
        tree.query(world_query, |entity| {
            found.insert(entity);
        });
        assert_eq!(found.len(), 2);
        assert!(found.contains(&e1));
        assert!(found.contains(&e2));

        tree.remove(l1);
        let mut found_after = HashSet::new();
        tree.query(world_query, |entity| {
            found_after.insert(entity);
        });
        assert_eq!(found_after.len(), 1);
        assert!(found_after.contains(&e2));

        tree.remove(l2);
        let mut found_final = Vec::new();
        tree.query(world_query, |entity| {
            found_final.push(entity);
        });
        assert!(found_final.is_empty());
    }

    #[test]
    fn multiple_updates_sequence_test() {
        let mut rng = StdRng::seed_from_u64(0xDEAD_BEEF_1234_5678);
        let mut tree = DynamicAabbTree::default();
        let mut center = Vec3::new(0.0, 0.0, 0.0);
        let mut aabb = make_aabb(center, 1.0);
        let leaf = tree.allocate_leaf(Entity::from_bits(200), aabb);

        for _ in 0..75 {
            center += Vec3::new(
                rng.random_range(-0.5..0.5),
                rng.random_range(-0.5..0.5),
                rng.random_range(-0.5..0.5),
            );
            aabb = make_aabb(center, 1.0);
            tree.update(leaf, aabb);
            assert_tree_invariants(&tree, true);
        }
    }

    #[test]
    fn worst_case_insertion_order_test() {
        let mut tree = DynamicAabbTree::default();
        let count = 128usize;
        for i in 0..count {
            let x = i as f32 * 1.5;
            tree.allocate_leaf(
                Entity::from_bits((i + 500) as u64),
                make_aabb(Vec3::new(x, 0.0, 0.0), 0.5),
            );
        }

        let (height, leaves) = assert_tree_invariants(&tree, true);
        assert_eq!(leaves.len(), count);
        let max_height = (count as f32).log2().ceil() as i32 + 2;
        assert!(
            height <= max_height,
            "height {height} exceeds bound {max_height}"
        );
    }

    #[test]
    fn stress_random_test() {
        let mut rng = StdRng::seed_from_u64(0xAABB_CCDD_1234_5678);
        let mut tree = DynamicAabbTree::default();
        let mut leaves: Vec<(NodeId, Entity, AABB)> = Vec::new();

        // Insert 300 random leaves
        for i in 0..300u64 {
            let center = Vec3::new(
                rng.random_range(-50.0..50.0),
                rng.random_range(-50.0..50.0),
                rng.random_range(-50.0..50.0),
            );
            let half = rng.random_range(0.2..3.0);
            let aabb = make_aabb(center, half);
            let entity = Entity::from_bits(i + 1000);
            let id = tree.allocate_leaf(entity, aabb);
            leaves.push((id, entity, aabb));
        }

        // Check tree structure, allow small imbalance
        let (_height, leaf_ids) = assert_tree_invariants(&tree, false);
        assert_eq!(leaf_ids.len(), leaves.len());

        // Remove ~1/3 of the leaves randomly
        let mut indices: Vec<usize> = (0..leaves.len()).collect();
        indices.shuffle(&mut rng);
        let remove_count = leaves.len() / 3;
        let mut removed = HashSet::new();
        for idx in indices.into_iter().take(remove_count) {
            let (id, _, _) = leaves[idx];
            tree.remove(id);
            removed.insert(id);
        }
        leaves.retain(|(id, _, _)| !removed.contains(id));

        // Check tree structure again after removals
        let (height, leaf_ids) = assert_tree_invariants(&tree, false);
        assert_eq!(leaf_ids.len(), leaves.len());

        // Optional: check that height is not degenerate
        let max_expected_height = (leaves.len() as f32).log2().ceil() as i32 + 5; // allow +5 for temporary imbalance
        assert!(
            height <= max_expected_height,
            "tree height too tall: {height}"
        );

        // Random queries to check correctness
        for _ in 0..100 {
            let center = Vec3::new(
                rng.random_range(-60.0..60.0),
                rng.random_range(-60.0..60.0),
                rng.random_range(-60.0..60.0),
            );
            let half = rng.random_range(0.5..5.0);
            let query = make_aabb(center, half);

            let mut found = HashSet::new();
            tree.query(query, |entity| {
                found.insert(entity);
            });

            let mut expected = HashSet::new();
            for (_, entity, aabb) in &leaves {
                let fat = DynamicAabbTree::expand_aabb(*aabb, 0.1);
                if fat.intersects(&query) {
                    expected.insert(*entity);
                }
            }

            assert_eq!(found, expected);
        }
    }

    #[test]
    fn stress_random_test_max_imbalance() {
        let mut rng = StdRng::seed_from_u64(0xAABB_CCDD_1234_5678);
        let mut tree = DynamicAabbTree::default();
        let mut leaves: Vec<(NodeId, Entity, AABB)> = Vec::new();

        // Insert 300 random leaves
        for i in 0..300u64 {
            let center = Vec3::new(
                rng.random_range(-50.0..50.0),
                rng.random_range(-50.0..50.0),
                rng.random_range(-50.0..50.0),
            );
            let half = rng.random_range(0.2..3.0);
            let aabb = make_aabb(center, half);
            let entity = Entity::from_bits(i + 1000);
            let id = tree.allocate_leaf(entity, aabb);
            leaves.push((id, entity, aabb));
        }

        // Helper to compute max imbalance recursively
        fn max_imbalance(tree: &DynamicAabbTree, node_id: NodeId) -> i32 {
            let node = &tree.nodes[node_id];
            if tree.is_leaf(node_id) {
                return 0;
            }

            let left = node.left.unwrap();
            let right = node.right.unwrap();
            let left_height = tree.nodes[left].height;
            let right_height = tree.nodes[right].height;

            let balance = (left_height - right_height).abs();
            let left_max = max_imbalance(tree, left);
            let right_max = max_imbalance(tree, right);

            balance.max(left_max).max(right_max)
        }

        // Initial max imbalance
        let initial_max = tree.root.map(|r| max_imbalance(&tree, r)).unwrap_or(0);
        println!("Max imbalance after insertion: {}", initial_max);
        assert!(
            initial_max <= 3,
            "initial max imbalance too high: {initial_max}"
        );

        // Remove ~1/3 of the leaves randomly
        let mut indices: Vec<usize> = (0..leaves.len()).collect();
        indices.shuffle(&mut rng);
        let remove_count = leaves.len() / 3;
        let mut removed = HashSet::new();
        for idx in indices.iter().take(remove_count) {
            let (id, _, _) = leaves[*idx];
            tree.remove(id);
            removed.insert(id);
        }
        leaves.retain(|(id, _, _)| !removed.contains(id));

        // Max imbalance after removals
        let post_remove_max = tree.root.map(|r| max_imbalance(&tree, r)).unwrap_or(0);
        println!("Max imbalance after removals: {}", post_remove_max);
        assert!(
            post_remove_max <= 3,
            "post-remove max imbalance too high: {post_remove_max}"
        );

        // Random queries to check correctness
        for _ in 0..100 {
            let center = Vec3::new(
                rng.random_range(-60.0..60.0),
                rng.random_range(-60.0..60.0),
                rng.random_range(-60.0..60.0),
            );
            let half = rng.random_range(0.5..5.0);
            let query = make_aabb(center, half);

            let mut found = HashSet::new();
            tree.query(query, |entity| {
                found.insert(entity);
            });

            let mut expected = HashSet::new();
            for (_, entity, aabb) in &leaves {
                let fat = DynamicAabbTree::expand_aabb(*aabb, 0.1);
                if fat.intersects(&query) {
                    expected.insert(*entity);
                }
            }

            assert_eq!(found, expected);
        }
    }
}
