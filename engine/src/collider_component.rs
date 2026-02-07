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

#[derive(Clone, Debug)]
pub struct BVHNode {
    pub aabb: AABB,
    pub left: Option<Box<BVHNode>>,
    pub right: Option<Box<BVHNode>>,
    pub triangles: Vec<Triangle>,
}

impl BVHNode {
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
}

#[derive(Clone, Copy, Debug)]
pub enum ConvexShape {
    Cuboid {
        length: f32,
        width: f32,
        height: f32,
    },
    Sphere {
        radius: f32,
    },
    Triangle {
        v0: Vec3,
        v1: Vec3,
        v2: Vec3,
    },
}

#[derive(Clone, Copy, Component, Debug)]
pub struct ConvexCollider {
    pub shape: ConvexShape,
    pub layer: CollisionLayer,
}

impl ConvexCollider {
    pub fn cuboid(size: Vec3, layer: CollisionLayer) -> Self {
        Self {
            shape: ConvexShape::Cuboid {
                length: size.x,
                width: size.y,
                height: size.z,
            },
            layer,
        }
    }

    pub fn cuboid_from_aabb(aabb: AABB, layer: CollisionLayer) -> Self {
        let size = aabb.max - aabb.min;
        Self::cuboid(size, layer)
    }

    pub fn cube(size: f32, layer: CollisionLayer) -> Self {
        Self::cuboid(Vec3::splat(size), layer)
    }

    pub fn sphere(radius: f32, layer: CollisionLayer) -> Self {
        Self {
            shape: ConvexShape::Sphere { radius },
            layer,
        }
    }

    pub fn triangle(v0: Vec3, v1: Vec3, v2: Vec3, layer: CollisionLayer) -> Self {
        Self {
            shape: ConvexShape::Triangle { v0, v1, v2 },
            layer,
        }
    }

    pub fn sphere_from_aabb(aabb: AABB, layer: CollisionLayer) -> Self {
        let center = (aabb.min + aabb.max) * 0.5;
        let radius = (aabb.max - center).length();
        Self::sphere(radius, layer)
    }

    pub fn as_cuboid(&self) -> Option<(f32, f32, f32)> {
        match self.shape {
            ConvexShape::Cuboid {
                length,
                width,
                height,
            } => Some((length, width, height)),
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
            ConvexShape::Cuboid {
                length,
                width,
                height,
            } => Vec3::new(
                if local_dir.x >= 0.0 {
                    length * 0.5
                } else {
                    -length * 0.5
                },
                if local_dir.y >= 0.0 {
                    width * 0.5
                } else {
                    -width * 0.5
                },
                if local_dir.z >= 0.0 {
                    height * 0.5
                } else {
                    -height * 0.5
                },
            ),
            ConvexShape::Sphere { radius } => {
                if local_dir.length_squared() <= SUPPORT_EPSILON {
                    Vec3::ZERO
                } else {
                    local_dir.normalize() * radius
                }
            }
            ConvexShape::Triangle { v0, v1, v2 } => {
                let mut best = v0;
                let mut best_dot = v0.dot(local_dir);
                let v1_dot = v1.dot(local_dir);
                if v1_dot > best_dot {
                    best = v1;
                    best_dot = v1_dot;
                }
                let v2_dot = v2.dot(local_dir);
                if v2_dot > best_dot {
                    best = v2;
                }
                best
            }
        };

        transform.transform_point3(local_point)
    }
}

impl Collider for ConvexCollider {
    fn aabb(&self, transform: &Mat4) -> AABB {
        match self.shape {
            ConvexShape::Cuboid {
                length,
                width,
                height,
            } => {
                let half_extents = Vec3::new(length * 0.5, width * 0.5, height * 0.5);
                let local_aabb = AABB {
                    min: -half_extents,
                    max: half_extents,
                };
                transform_aabb(local_aabb, transform)
            }
            ConvexShape::Sphere { radius } => {
                let center = transform.transform_point3(Vec3::ZERO);
                let scale = max_scale(transform);
                let radius = radius * scale;
                AABB {
                    min: center - Vec3::splat(radius),
                    max: center + Vec3::splat(radius),
                }
            }
            ConvexShape::Triangle { v0, v1, v2 } => {
                let local_min = v0.min(v1).min(v2);
                let local_max = v0.max(v1).max(v2);
                let local_aabb = AABB {
                    min: local_min,
                    max: local_max,
                };
                transform_aabb(local_aabb, transform)
            }
        }
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

fn max_scale(transform: &Mat4) -> f32 {
    let x = transform.x_axis.truncate().length();
    let y = transform.y_axis.truncate().length();
    let z = transform.z_axis.truncate().length();
    x.max(y).max(z)
}

pub(crate) fn closest_point_on_triangle(p: Vec3, tri: &Triangle) -> Vec3 {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_vec3_eq(actual: Vec3, expected: Vec3) {
        let diff = actual - expected;
        let eps = 1e-5;
        assert!(
            diff.x.abs() <= eps && diff.y.abs() <= eps && diff.z.abs() <= eps,
            "expected {:?}, got {:?}",
            expected,
            actual
        );
    }

    #[test]
    fn support_cuboid_identity_selects_corner() {
        let collider = ConvexCollider::cuboid(Vec3::new(2.0, 4.0, 6.0), CollisionLayer::Default);
        let transform = Mat4::IDENTITY;
        let dir = Vec3::new(1.0, -1.0, 1.0);

        let support = collider.support(transform, dir);

        assert_vec3_eq(support, Vec3::new(1.0, -2.0, 3.0));
    }

    #[test]
    fn support_cuboid_handles_translation() {
        let collider = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let transform = Mat4::from_translation(Vec3::new(10.0, -5.0, 3.0));
        let dir = Vec3::new(-1.0, 1.0, -1.0);

        let support = collider.support(transform, dir);

        assert_vec3_eq(support, Vec3::new(9.0, -4.0, 2.0));
    }

    #[test]
    fn support_cuboid_handles_rotation() {
        let collider = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let transform = Mat4::from_rotation_z(std::f32::consts::FRAC_PI_2);
        let dir = Vec3::X;

        let support = collider.support(transform, dir);

        assert_vec3_eq(support, Vec3::new(1.0, -1.0, 1.0));
    }

    #[test]
    fn support_sphere_identity_matches_direction() {
        let collider = ConvexCollider::sphere(2.5, CollisionLayer::Default);
        let transform = Mat4::IDENTITY;
        let dir = Vec3::new(3.0, 4.0, 0.0);

        let support = collider.support(transform, dir);

        let expected = dir.normalize() * 2.5;
        assert_vec3_eq(support, expected);
    }

    #[test]
    fn support_sphere_handles_zero_direction() {
        let collider = ConvexCollider::sphere(2.5, CollisionLayer::Default);
        let transform = Mat4::from_translation(Vec3::new(1.0, 2.0, 3.0));
        let dir = Vec3::ZERO;

        let support = collider.support(transform, dir);

        assert_vec3_eq(support, Vec3::new(1.0, 2.0, 3.0));
    }
}
