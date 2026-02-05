use bevy_ecs::component::Component;
use glam::{Mat4, Vec3};

use crate::handles::RenderBodyHandle;
use crate::mesh::AABB;

#[derive(Clone, Copy, Debug)]
pub enum CollisionLayer {
    Default,
    Player,
    Enemy,
    Environment,
}

const SUPPORT_EPSILON: f32 = 1e-6;


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

#[derive(Clone, Copy, Debug)]
pub enum ConvexShape {
    Cuboid { aabb: AABB },
    Sphere { radius: f32 },
}

#[derive(Clone, Copy, Component, Debug)]
pub struct ConvexCollider {
    pub shape: ConvexShape,
    pub layer: CollisionLayer,
}

impl ConvexCollider {
    pub fn cuboid(aabb: AABB, layer: CollisionLayer) -> Self {
        Self {
            shape: ConvexShape::Cuboid { aabb },
            layer,
        }
    }

    pub fn cube(size: f32, layer: CollisionLayer) -> Self {
        let half = size * 0.5;
        let aabb = AABB {
            min: Vec3::splat(-half),
            max: Vec3::splat(half),
        };
        Self::cuboid(aabb, layer)
    }

    pub fn sphere(radius: f32, layer: CollisionLayer) -> Self {
        Self {
            shape: ConvexShape::Sphere { radius },
            layer,
        }
    }

    pub fn sphere_from_aabb(aabb: AABB, layer: CollisionLayer) -> Self {
        let center = (aabb.min + aabb.max) * 0.5;
        let radius = (aabb.max - center).length();
        Self::sphere(radius, layer)
    }

    pub fn as_cuboid(&self) -> Option<AABB> {
        match self.shape {
            ConvexShape::Cuboid { aabb } => Some(aabb),
            _ => None,
        }
    }

    pub fn as_sphere_radius(&self) -> Option<f32> {
        match self.shape {
            ConvexShape::Sphere { radius } => Some(radius),
            _ => None,
        }
    }

    pub fn support(&self, transform: Mat4, dir_world: Vec3) -> Vec3 {
        let local_dir = if dir_world.length_squared() <= SUPPORT_EPSILON {
            Vec3::ZERO
        } else {
            transform.inverse().transform_vector3(dir_world)
        };

        let local_point = match self.shape {
            ConvexShape::Cuboid { aabb } => Vec3::new(
                if local_dir.x >= 0.0 { aabb.max.x } else { aabb.min.x },
                if local_dir.y >= 0.0 { aabb.max.y } else { aabb.min.y },
                if local_dir.z >= 0.0 { aabb.max.z } else { aabb.min.z },
            ),
            ConvexShape::Sphere { radius } => {
                if local_dir.length_squared() <= SUPPORT_EPSILON {
                    Vec3::ZERO
                } else {
                    local_dir.normalize() * radius
                }
            }
        };

        transform.transform_point3(local_point)
    }
}

impl Collider for ConvexCollider {
    fn aabb(&self, transform: &Mat4) -> AABB {
        match self.shape {
            ConvexShape::Cuboid { aabb } => transform_aabb(aabb, transform),
            ConvexShape::Sphere { radius } => {
                let center = transform.transform_point3(Vec3::ZERO);
                let scale = max_scale(transform);
                let radius = radius * scale;
                AABB {
                    min: center - Vec3::splat(radius),
                    max: center + Vec3::splat(radius),
                }
            }
        }
    }

    fn collide_triangle(&self, tri: &Triangle, transform: &Mat4) -> Option<CollisionHit> {
        // Transform convex center into world space
        let center = transform.transform_point3(Vec3::ZERO);

        // Step 1: Closest point on triangle to convex center
        let closest = closest_point_on_triangle(center, tri);

        // Step 2: Direction from triangle to convex
        let mut dir = center - closest;
        let dist_sq = dir.length_squared();

        // Fallback: if center is exactly on triangle plane, use triangle normal
        if dist_sq <= f32::EPSILON {
            dir = tri.normal()?;
        } else {
            dir /= dist_sq.sqrt();
        }

        // Step 3: Use your support function in world space
        let support_world = self.support(*transform, -dir);

        // Step 4: Compute penetration along normal
        let penetration_vec = support_world - closest;
        let penetration = penetration_vec.dot(dir);

        if penetration <= 0.0 {
            return None;
        }

        Some(CollisionHit {
            normal: dir,
            penetration,
        })
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

fn max_scale(transform: &Mat4) -> f32 {
    let x = transform.x_axis.truncate().length();
    let y = transform.y_axis.truncate().length();
    let z = transform.z_axis.truncate().length();
    x.max(y).max(z)
}

fn closest_point_on_triangle(p: Vec3, tri: &Triangle) -> Vec3 {
    // Real-Time Collision Detection (Christer Ericson)
    let ab = tri.v1 - tri.v0;
    let ac = tri.v2 - tri.v0;
    let ap = p - tri.v0;

    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return tri.v0;
    }

    let bp = p - tri.v1;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return tri.v1;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return tri.v0 + ab * v;
    }

    let cp = p - tri.v2;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return tri.v2;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return tri.v0 + ac * w;
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return tri.v1 + (tri.v2 - tri.v1) * w;
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    tri.v0 + ab * v + ac * w
}
