use bevy_ecs::component::Component;
use glam::{Mat4, Vec3};

use crate::handles::RenderBodyHandle;
use crate::mesh::AABB;

#[derive(Clone, Copy)]
pub enum CollisionLayer {
    Default,
    Player,
    Enemy,
    Environment,
}


#[derive(Clone, Debug)]
pub struct Triangle {
    pub v0: Vec3,
    pub v1: Vec3,
    pub v2: Vec3,
}

impl Triangle {
    fn aabb(&self) -> AABB {
        let min = self.v0.min(self.v1).min(self.v2);
        let max = self.v0.max(self.v1).max(self.v2);
        AABB { min, max }
    }

    fn normal(&self) -> Option<Vec3> {
        let n = (self.v1 - self.v0).cross(self.v2 - self.v0);
        let len = n.length();
        if len <= f32::EPSILON {
            None
        } else {
            Some(n / len)
        }
    }
}

#[derive(Clone, Copy)]
pub struct CollisionHit {
    pub normal: Vec3,
    pub penetration: f32,
}

#[derive(Clone, Debug)]
pub struct BVHNode {
    pub aabb: AABB,
    pub left: Option<Box<BVHNode>>,
    pub right: Option<Box<BVHNode>>,
    pub triangles: Vec<Triangle>,
}

impl BVHNode {
    pub fn query_collider<C: Collider>(
        &self,
        collider: &C,
        collider_transform: &Mat4,
        hits: &mut Vec<(Triangle, CollisionHit)>,
    ) {
        if !aabb_intersects(&self.aabb, &collider.aabb(collider_transform)) {
            return;
        }

        if self.left.is_none() && self.right.is_none() {
            for tri in &self.triangles {
                if let Some(hit) = collider.collide_triangle(tri, collider_transform) {
                    hits.push((tri.clone(), hit));
                }
            }
        } else {
            if let Some(left) = &self.left {
                left.query_collider(collider, collider_transform, hits);
            }
            if let Some(right) = &self.right {
                right.query_collider(collider, collider_transform, hits);
            }
        }
    }

    pub fn build(triangles: Vec<Triangle>, max_leaf_size: usize) -> Self {
        let mut min = triangles[0].v0;
        let mut max = triangles[0].v0;
        for tri in &triangles {
            min = min.min(tri.v0).min(tri.v1).min(tri.v2);
            max = max.max(tri.v0).max(tri.v1).max(tri.v2);
        }
        let aabb = AABB { min, max };

        if triangles.len() <= max_leaf_size {
            return BVHNode {
                aabb,
                left: None,
                right: None,
                triangles,
            };
        }

        let extent = max - min;
        let axis = if extent.x > extent.y && extent.x > extent.z {
            0
        } else if extent.y > extent.z {
            1
        } else {
            2
        };

        let mut sorted = triangles.clone();
        sorted.sort_by(|a, b| {
            let ca = (a.v0 + a.v1 + a.v2) / 3.0;
            let cb = (b.v0 + b.v1 + b.v2) / 3.0;
            ca[axis].partial_cmp(&cb[axis]).unwrap()
        });

        let mid = sorted.len() / 2;
        let left = BVHNode::build(sorted[..mid].to_vec(), max_leaf_size);
        let right = BVHNode::build(sorted[mid..].to_vec(), max_leaf_size);

        BVHNode {
            aabb,
            left: Some(Box::new(left)),
            right: Some(Box::new(right)),
            triangles: vec![],
        }
    }
}


pub trait Collider {
    fn aabb(&self, transform: &Mat4) -> AABB;
    fn collide_triangle(&self, tri: &Triangle, transform: &Mat4) -> Option<CollisionHit>;
}

#[derive(Clone, Copy, Component)]
pub struct BoxCollider {
    pub aabb: AABB,
    pub layer: CollisionLayer,
}

impl BoxCollider {
    pub fn new(aabb: AABB, layer: CollisionLayer) -> Self {
        Self { aabb, layer }
    }

    #[allow(dead_code)]
    pub fn intersects(&self, other: &BoxCollider) -> bool {
        (self.aabb.min.x <= other.aabb.max.x && self.aabb.max.x >= other.aabb.min.x)
            && (self.aabb.min.y <= other.aabb.max.y && self.aabb.max.y >= other.aabb.min.y)
            && (self.aabb.min.z <= other.aabb.max.z && self.aabb.max.z >= other.aabb.min.z)
    }
}

impl Collider for BoxCollider {
    fn aabb(&self, transform: &Mat4) -> AABB {
        transform_aabb(self.aabb, transform)
    }

    fn collide_triangle(&self, tri: &Triangle, transform: &Mat4) -> Option<CollisionHit> {
        let collider_aabb = self.aabb(transform);
        let tri_aabb = tri.aabb();
        if !aabb_intersects(&collider_aabb, &tri_aabb) {
            return None;
        }
        let normal = tri.normal()?;

        let corners = aabb_corners(&collider_aabb);
        let mut min_d = f32::INFINITY;
        let mut max_d = f32::NEG_INFINITY;
        for corner in &corners {
            let d = (*corner - tri.v0).dot(normal);
            min_d = min_d.min(d);
            max_d = max_d.max(d);
        }

        if min_d > 0.0 || max_d < 0.0 {
            return None;
        }

        let penetration = (-min_d).max(0.0);
        if penetration <= 0.0 {
            return None;
        }

        Some(CollisionHit { normal, penetration })
    }
}

#[derive(Clone, Copy, Component)]
pub struct MeshCollider {
    pub render_body_id: RenderBodyHandle,
    pub layer: CollisionLayer,
}

impl MeshCollider {
    pub fn new(render_body_id: RenderBodyHandle, layer: CollisionLayer) -> Self {
        Self {
            render_body_id,
            layer,
        }
    }
}

fn transform_aabb(local: AABB, transform: &Mat4) -> AABB {
    let min = local.min;
    let max = local.max;

    let corners = [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(max.x, max.y, max.z),
    ];

    let mut world_min = transform.transform_point3(corners[0]);
    let mut world_max = world_min;

    for corner in corners.iter().skip(1) {
        let world = transform.transform_point3(*corner);
        world_min = world_min.min(world);
        world_max = world_max.max(world);
    }

    AABB {
        min: world_min,
        max: world_max,
    }
}

fn aabb_corners(aabb: &AABB) -> [Vec3; 8] {
    let min = aabb.min;
    let max = aabb.max;
    [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(max.x, max.y, max.z),
    ]
}

fn aabb_intersects(a: &AABB, b: &AABB) -> bool {
    (a.min.x <= b.max.x && a.max.x >= b.min.x)
        && (a.min.y <= b.max.y && a.max.y >= b.min.y)
        && (a.min.z <= b.max.z && a.max.z >= b.min.z)
}
