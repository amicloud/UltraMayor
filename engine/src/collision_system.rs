use bevy_ecs::prelude::{Changed, Entity, Query, Res, ResMut};
use glam::{Mat4, Vec3};

use crate::{
    collider_component::{closest_point_on_triangle, BVHNode, Collider, ConvexCollider, ConvexShape, MeshCollider, Triangle},
    epa::epa,
    gjk::{gjk_intersect, GjkResult},
    mesh::AABB,
    physics_resource::{Contact, PhysicsResource},
    render_resource_manager::RenderResourceManager,
    velocity_component::VelocityComponent,
    TransformComponent,
};

#[derive(Default)]
pub struct CollisionSystem {}

impl CollisionSystem {
    pub fn update_world_aabb_cache(
        query: Query<
            (
                Entity,
                &TransformComponent,
                Option<&ConvexCollider>,
                Option<&MeshCollider>,
            ),
            Changed<TransformComponent>,
        >,
        render_resources: Res<RenderResourceManager>,
        mut phys: ResMut<PhysicsResource>,
    ) {
        for (entity, transform, convex_collider, mesh_collider) in &query {
            if let Some(mesh_collider) = mesh_collider {
                if let Some(local_aabb) =
                    render_body_local_aabb(mesh_collider.render_body_id, &render_resources)
                {
                    let world_aabb = transform_aabb(local_aabb, transform);
                    phys.world_aabbs.insert(entity, world_aabb);
                    continue;
                }
            }

            if let Some(convex_collider) = convex_collider {
                let world_aabb = convex_collider.aabb(&transform.to_mat4());
                phys.world_aabbs.insert(entity, world_aabb);
            }
        }
    }

    pub fn generate_contacts(
        moving_query: Query<
            (
                Entity,
                &TransformComponent,
                Option<&VelocityComponent>,
                Option<&ConvexCollider>,
                Option<&MeshCollider>,
            ),
            Changed<TransformComponent>,
        >,
        all_query: Query<(
            Entity,
            &TransformComponent,
            Option<&VelocityComponent>,
            Option<&ConvexCollider>,
            Option<&MeshCollider>,
        )>,
        render_resources: Res<RenderResourceManager>,
        mut phys: ResMut<PhysicsResource>,
    ) {
        // Collect moving entities
        let moving_entities: Vec<Entity> = moving_query.iter().map(|(e, _, _, _, _)| e).collect();
        let mut contacts = Vec::new();

        // Iterate over moving entities only
        for &entity_a in &moving_entities {
            let Some(aabb_a) = phys.world_aabbs.get(&entity_a) else {
                continue;
            };

            let Ok((_, transform_a, velocity_a, convex_a, mesh_a)) = moving_query.get(entity_a)
            else {
                continue;
            };

            // Compare against all colliders (moving + static)
            for (entity_b, transform_b, velocity_b, convex_b, mesh_b) in &all_query {
                if entity_a == entity_b {
                    continue;
                } // skip self

                let Some(aabb_b) = phys.world_aabbs.get(&entity_b) else {
                    continue;
                };

                let mut aabb_a_swept = *aabb_a;
                if let Some(velocity) = velocity_a {
                    let delta = velocity.translational * fixed_dt();
                    if delta.length_squared() > 0.0 {
                        aabb_a_swept = swept_aabb(aabb_a, delta);
                    }
                }

                let mut aabb_b_swept = *aabb_b;
                if let Some(velocity) = velocity_b {
                    let delta = velocity.translational * fixed_dt();
                    if delta.length_squared() > 0.0 {
                        aabb_b_swept = swept_aabb(aabb_b, delta);
                    }
                }

                if aabb_intersects(&aabb_a_swept, &aabb_b_swept) {
                    if let (Some(convex_a), Some(convex_b)) = (convex_a, convex_b) {
                        if let Some(contact) = convex_convex_contact(
                            entity_a,
                            convex_a,
                            transform_a,
                            entity_b,
                            convex_b,
                            transform_b,
                        ) {
                            contacts.push(contact);
                        }
                        continue;
                    }

                    if let (Some(convex_a), Some(mesh_b)) = (convex_a, mesh_b) {
                        let mesh_contacts = convex_mesh_contact(
                            entity_a,
                            convex_a,
                            transform_a,
                            velocity_a,
                            entity_b,
                            mesh_b,
                            transform_b,
                            &render_resources,
                        );
                        contacts.extend(mesh_contacts);
                        continue;
                    }

                    if let (Some(mesh_a), Some(convex_b)) = (mesh_a, convex_b) {
                        let mesh_contacts = convex_mesh_contact(
                            entity_b,
                            convex_b,
                            transform_b,
                            velocity_b,
                            entity_a,
                            mesh_a,
                            transform_a,
                            &render_resources,
                        );
                        contacts.extend(mesh_contacts);
                        continue;
                    }
                }
            }
        }

        for contact in contacts {
            phys.add_contact(contact);
        }
    }
}

fn transform_aabb(local: AABB, transform: &TransformComponent) -> AABB {
    let matrix = transform.to_mat4();
    transform_aabb_with_mat4(local, &matrix)
}

fn transform_aabb_with_mat4(local: AABB, transform: &Mat4) -> AABB {
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

fn aabb_intersects(a: &AABB, b: &AABB) -> bool {
    (a.min.x <= b.max.x && a.max.x >= b.min.x)
        && (a.min.y <= b.max.y && a.max.y >= b.min.y)
        && (a.min.z <= b.max.z && a.max.z >= b.min.z)
}

fn sphere_sphere_contact(
    entity_a: Entity,
    center_a: Vec3,
    radius_a: f32,
    entity_b: Entity,
    center_b: Vec3,
    radius_b: f32,
) -> Option<Contact> {
    let ab = center_b - center_a;
    let distance_sq = ab.length_squared();
    let radius_sum = radius_a + radius_b;
    let radius_sum_sq = radius_sum * radius_sum;

    if distance_sq >= radius_sum_sq {
        return None;
    }

    let distance = distance_sq.sqrt();
    let penetration = radius_sum - distance;

    let normal = if distance > f32::EPSILON {
        ab / distance
    } else {
        Vec3::X
    };

    Some(Contact {
        entity_a,
        entity_b,
        normal,
        penetration,
    })
}

fn convex_convex_contact(
    entity_a: Entity,
    collider_a: &ConvexCollider,
    transform_a: &TransformComponent,
    entity_b: Entity,
    collider_b: &ConvexCollider,
    transform_b: &TransformComponent,
) -> Option<Contact> {
    let a_world = transform_a.to_mat4();
    let b_world = transform_b.to_mat4();

    match collider_a.shape {
        ConvexShape::Sphere { radius: ra } => match collider_b.shape {
            ConvexShape::Sphere { radius: rb } => {
                return sphere_sphere_contact(
                    entity_a,
                    a_world.transform_point3(Vec3::ZERO),
                    ra,
                    entity_b,
                    b_world.transform_point3(Vec3::ZERO),
                    rb,
                );
            }
            _ => {}
        },
        _ => {}
    }

    let result = gjk_intersect(collider_a, a_world, collider_b, b_world);
    let simplex = match result {
        GjkResult::Intersection(hit) => hit.simplex,
        GjkResult::NoIntersection => return None,
    };

    let epa_result = epa(collider_a, a_world, collider_b, b_world, &simplex)?;

    let mut normal = epa_result.normal;
    let a_center = a_world.transform_point3(Vec3::ZERO);
    let b_center = b_world.transform_point3(Vec3::ZERO);
    let ab = b_center - a_center;
    if ab.length_squared() > f32::EPSILON && normal.dot(ab) < 0.0 {
        normal = -normal;
    }

    Some(Contact {
        entity_a,
        entity_b,
        normal,
        penetration: epa_result.penetration_depth,
    })
}

fn convex_mesh_contact(
    convex_entity: Entity,
    convex_collider: &ConvexCollider,
    convex_transform: &TransformComponent,
    convex_velocity: Option<&VelocityComponent>,
    mesh_entity: Entity,
    mesh_collider: &MeshCollider,
    mesh_transform: &TransformComponent,
    render_resources: &RenderResourceManager,
) -> Vec<Contact> {
    let Some(render_body) = render_resources
        .render_body_manager
        .get_render_body(mesh_collider.render_body_id)
    else {
        return Vec::new();
    };

    let convex_world = convex_transform.to_mat4();
    let mesh_entity_world = mesh_transform.to_mat4();

    let convex_aabb_world = convex_collider.aabb(&convex_world);
    let mut candidates: Vec<ContactCandidate> = Vec::new();

    for part in &render_body.parts {
        let Some(mesh) = render_resources.mesh_manager.get_mesh(part.mesh_id) else {
            continue;
        };
        let Some(bvh) = mesh.bvh.as_ref() else {
            continue;
        };

        let mesh_world = mesh_entity_world * part.local_transform;
        let Some(mesh_world_inv) = mesh_world.try_inverse() else {
            continue;
        };

        candidates.extend(convex_mesh_contact_at_transform(
            convex_collider,
            convex_world,
            &mesh_world,
            &mesh_world_inv,
            bvh,
        ));

        if let Some(velocity) = convex_velocity {
            let delta = velocity.translational * fixed_dt();
            let distance = delta.length();
            if distance > 0.0 {
                let step = (convex_aabb_world.max - convex_aabb_world.min)
                    .abs()
                    .min_element()
                    .max(0.01)
                    * 0.5;
                let steps = ((distance / step).ceil() as i32).clamp(1, 20);

                for i in 1..=steps {
                    let t = i as f32 / steps as f32;
                    let swept_position = convex_transform.position + delta * t;
                    let swept_transform = TransformComponent {
                        position: swept_position,
                        rotation: convex_transform.rotation,
                        scale: convex_transform.scale,
                    };
                    let swept_world = swept_transform.to_mat4();
                    candidates.extend(convex_mesh_contact_at_transform(
                        convex_collider,
                        swept_world,
                        &mesh_world,
                        &mesh_world_inv,
                        bvh,
                    ));
                }
            }
        }
    }

    reduce_contact_candidates(
        mesh_entity,
        convex_entity,
        candidates,
        convex_aabb_world,
    )
}

fn convex_mesh_contact_at_transform(
    convex_collider: &ConvexCollider,
    convex_world: Mat4,
    mesh_world: &Mat4,
    mesh_world_inv: &Mat4,
    bvh: &BVHNode,
) -> Vec<ContactCandidate> {
    let collider_in_mesh_space = *mesh_world_inv * convex_world;
    let _convex_center_mesh = collider_in_mesh_space.transform_point3(Vec3::ZERO);
    let convex_aabb_mesh = convex_collider.aabb(&collider_in_mesh_space);

    let mut triangles = Vec::new();
    collect_triangles_in_aabb(bvh, &convex_aabb_mesh, &mut triangles);
    if triangles.is_empty() {
        return Vec::new();
    }

    let convex_center_world = convex_world.transform_point3(Vec3::ZERO);

    let mut candidates = Vec::new();
    for tri in triangles {
        let tri_world = Triangle {
            v0: mesh_world.transform_point3(tri.v0),
            v1: mesh_world.transform_point3(tri.v1),
            v2: mesh_world.transform_point3(tri.v2),
        };

        let closest_world = closest_point_on_triangle(convex_center_world, &tri_world);

        let face_normal_world = {
            let n = (tri_world.v1 - tri_world.v0).cross(tri_world.v2 - tri_world.v0);
            if n.length_squared() > f32::EPSILON {
                Some(n.normalize())
            } else {
                None
            }
        };

        let feature_normal_world = {
            let n = convex_center_world - closest_world;
            if n.length_squared() > f32::EPSILON {
                Some(n.normalize())
            } else {
                None
            }
        };

        let is_sphere = matches!(convex_collider.shape, ConvexShape::Sphere { .. });
        let prefer_face_normal = is_sphere || is_face_center(&tri_world, closest_world);

        let face_candidate = face_normal_world.and_then(|normal_world| {
            let mut normal = normal_world;
            if (convex_center_world - closest_world).dot(normal) < 0.0 {
                normal = -normal;
            }
            let support_world = convex_collider.support(convex_world, normal);
            let penetration = (support_world - closest_world).dot(normal);
            if penetration <= 0.0 {
                None
            } else {
                Some(ContactCandidate {
                    point: closest_world,
                    normal,
                    penetration,
                })
            }
        });

        let feature_candidate = feature_normal_world.and_then(|normal_world| {
            let mut normal = normal_world;
            if (convex_center_world - closest_world).dot(normal) < 0.0 {
                normal = -normal;
            }
            let support_world = convex_collider.support(convex_world, normal);
            let penetration = (support_world - closest_world).dot(normal);
            if penetration <= 0.0 {
                None
            } else {
                Some(ContactCandidate {
                    point: closest_world,
                    normal,
                    penetration,
                })
            }
        });

        let best = if prefer_face_normal {
            face_candidate.or(feature_candidate)
        } else {
            match (face_candidate, feature_candidate) {
                (Some(face), Some(feature)) => {
                    if feature.penetration > face.penetration {
                        Some(feature)
                    } else {
                        Some(face)
                    }
                }
                (Some(face), None) => Some(face),
                (None, Some(feature)) => Some(feature),
                (None, None) => None,
            }
        };

        if let Some(candidate) = best {
            candidates.push(candidate);
        }
    }

    candidates
}

#[derive(Clone, Copy, Debug)]
struct ContactCandidate {
    point: Vec3,
    normal: Vec3,
    penetration: f32,
}

fn reduce_contact_candidates(
    mesh_entity: Entity,
    convex_entity: Entity,
    mut candidates: Vec<ContactCandidate>,
    convex_aabb_world: AABB,
) -> Vec<Contact> {
    candidates.retain(|c| c.penetration > 0.0 && c.normal.length_squared() > f32::EPSILON);
    if candidates.is_empty() {
        return Vec::new();
    }

    candidates.sort_by(|a, b| {
        b.penetration
            .total_cmp(&a.penetration)
            .then_with(|| a.point.x.total_cmp(&b.point.x))
            .then_with(|| a.point.y.total_cmp(&b.point.y))
            .then_with(|| a.point.z.total_cmp(&b.point.z))
            .then_with(|| a.normal.x.total_cmp(&b.normal.x))
            .then_with(|| a.normal.y.total_cmp(&b.normal.y))
            .then_with(|| a.normal.z.total_cmp(&b.normal.z))
    });

    let extent = convex_aabb_world.max - convex_aabb_world.min;
    let cluster_distance = extent.length().max(0.01) * 0.1;

    let mut selected: Vec<ContactCandidate> = Vec::new();
    for candidate in candidates {
        if selected
            .iter()
            .any(|c| (c.point - candidate.point).length() < cluster_distance)
        {
            continue;
        }
        selected.push(candidate);
        if selected.len() >= 4 {
            break;
        }
    }

    selected
        .into_iter()
        .map(|candidate| Contact {
            entity_a: mesh_entity,
            entity_b: convex_entity,
            normal: candidate.normal.normalize(),
            penetration: candidate.penetration,
        })
        .collect()
}

fn collect_triangles_in_aabb(bvh: &BVHNode, target: &AABB, out: &mut Vec<Triangle>) {
    if !aabb_intersects(&bvh.aabb, target) {
        return;
    }

    if bvh.left.is_none() && bvh.right.is_none() {
        for tri in &bvh.triangles {
            let tri_aabb = triangle_aabb(tri);
            if aabb_intersects(&tri_aabb, target) {
                out.push(tri.clone());
            }
        }
        return;
    }

    if let Some(left) = &bvh.left {
        collect_triangles_in_aabb(left, target, out);
    }
    if let Some(right) = &bvh.right {
        collect_triangles_in_aabb(right, target, out);
    }
}

fn triangle_aabb(tri: &Triangle) -> AABB {
    let min = tri.v0.min(tri.v1).min(tri.v2);
    let max = tri.v0.max(tri.v1).max(tri.v2);
    AABB { min, max }
}

fn is_face_center(tri: &Triangle, point: Vec3) -> bool {
    let e0 = tri.v1 - tri.v0;
    let e1 = tri.v2 - tri.v1;
    let e2 = tri.v0 - tri.v2;

    let p0 = point - tri.v0;
    let p1 = point - tri.v1;
    let p2 = point - tri.v2;

    let dist0 = (p0 - e0 * (p0.dot(e0) / e0.length_squared().max(f32::EPSILON))).length();
    let dist1 = (p1 - e1 * (p1.dot(e1) / e1.length_squared().max(f32::EPSILON))).length();
    let dist2 = (p2 - e2 * (p2.dot(e2) / e2.length_squared().max(f32::EPSILON))).length();

    let edge_min = e0.length().min(e1.length()).min(e2.length()).max(0.01);
    let threshold = edge_min * 0.15;

    dist0 > threshold && dist1 > threshold && dist2 > threshold
}

fn swept_aabb(aabb: &AABB, delta: Vec3) -> AABB {
    let moved_min = aabb.min + delta;
    let moved_max = aabb.max + delta;
    AABB {
        min: aabb.min.min(moved_min),
        max: aabb.max.max(moved_max),
    }
}

fn fixed_dt() -> f32 {
    1.0 / 60.0
}

fn render_body_local_aabb(
    render_body_id: crate::handles::RenderBodyHandle,
    render_resources: &RenderResourceManager,
) -> Option<AABB> {
    let render_body = render_resources
        .render_body_manager
        .get_render_body(render_body_id)?;

    let mut combined: Option<AABB> = None;
    for part in &render_body.parts {
        let mesh = render_resources.mesh_manager.get_mesh(part.mesh_id)?;
        let part_aabb = transform_aabb_with_mat4(mesh.aabb, &part.local_transform);
        combined = Some(match combined {
            Some(existing) => union_aabb(existing, part_aabb),
            None => part_aabb,
        });
    }

    combined
}

fn union_aabb(a: AABB, b: AABB) -> AABB {
    AABB {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use glam::{Mat4, Quat, Vec3};

    fn make_triangle() -> Triangle {
        Triangle {
            v0: Vec3::new(0.0, 0.0, 0.0),
            v1: Vec3::new(1.0, 0.0, 0.0),
            v2: Vec3::new(0.0, 1.0, 0.0),
        }
    }

    #[test]
    fn convex_mesh_contact_at_transform_hits_triangle() {
        let tri = make_triangle();
        let bvh = BVHNode::build(vec![tri], 4);

        let convex_collider = ConvexCollider::sphere(1.0, crate::collider_component::CollisionLayer::Default);
        let convex_transform = TransformComponent {
            position: Vec3::new(0.2, 0.2, 0.5),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        };
        let convex_world = convex_transform.to_mat4();

        let mesh_world = Mat4::IDENTITY;
        let mesh_world_inv = mesh_world.inverse();

        let contacts = convex_mesh_contact_at_transform(
            &convex_collider,
            convex_world,
            &mesh_world,
            &mesh_world_inv,
            &bvh,
        );

        assert_eq!(contacts.len(), 1);
        let contact = contacts[0];
        assert_relative_eq!(contact.normal.x, 0.0, epsilon = 1e-4);
        assert_relative_eq!(contact.normal.y, 0.0, epsilon = 1e-4);
        assert_relative_eq!(contact.normal.z, 1.0, epsilon = 1e-4);
        assert_relative_eq!(contact.penetration, 1.5, epsilon = 1e-4);
    }

    #[test]
    fn convex_mesh_contact_at_transform_flips_normal_toward_convex() {
        let tri = make_triangle();
        let bvh = BVHNode::build(vec![tri], 4);

        let convex_collider = ConvexCollider::sphere(1.0, crate::collider_component::CollisionLayer::Default);
        let convex_transform = TransformComponent {
            position: Vec3::new(0.2, 0.2, -0.5),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        };
        let convex_world = convex_transform.to_mat4();

        let mesh_world = Mat4::from_scale(Vec3::new(2.0, 1.0, 0.5));
        let mesh_world_inv = mesh_world.inverse();

        let contacts = convex_mesh_contact_at_transform(
            &convex_collider,
            convex_world,
            &mesh_world,
            &mesh_world_inv,
            &bvh,
        );

        assert_eq!(contacts.len(), 1);
        let contact = contacts[0];
        assert!(contact.normal.z < 0.0);
        assert_relative_eq!(contact.normal.length(), 1.0, epsilon = 1e-4);
        assert!(contact.penetration > 0.0);
    }

    #[test]
    fn reduce_contact_candidates_caps_to_four() {
        let mesh_entity = Entity::from_bits(1);
        let convex_entity = Entity::from_bits(2);
        let convex_aabb_world = AABB {
            min: Vec3::splat(-1.0),
            max: Vec3::splat(1.0),
        };

        let candidates = vec![
            ContactCandidate { point: Vec3::new(0.0, 0.0, 0.0), normal: Vec3::Z, penetration: 6.0 },
            ContactCandidate { point: Vec3::new(10.0, 0.0, 0.0), normal: Vec3::Z, penetration: 5.0 },
            ContactCandidate { point: Vec3::new(20.0, 0.0, 0.0), normal: Vec3::Z, penetration: 4.0 },
            ContactCandidate { point: Vec3::new(30.0, 0.0, 0.0), normal: Vec3::Z, penetration: 3.0 },
            ContactCandidate { point: Vec3::new(40.0, 0.0, 0.0), normal: Vec3::Z, penetration: 2.0 },
            ContactCandidate { point: Vec3::new(50.0, 0.0, 0.0), normal: Vec3::Z, penetration: 1.0 },
        ];

        let contacts = reduce_contact_candidates(mesh_entity, convex_entity, candidates, convex_aabb_world);

        assert_eq!(contacts.len(), 4);
        assert_relative_eq!(contacts[0].penetration, 6.0, epsilon = 1e-6);
        assert_relative_eq!(contacts[1].penetration, 5.0, epsilon = 1e-6);
        assert_relative_eq!(contacts[2].penetration, 4.0, epsilon = 1e-6);
        assert_relative_eq!(contacts[3].penetration, 3.0, epsilon = 1e-6);
    }
}
