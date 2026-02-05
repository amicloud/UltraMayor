use bevy_ecs::prelude::{Changed, Entity, Query, Res, ResMut};
use glam::{Mat4, Vec3};

use crate::{
    collider_component::{Collider, ConvexCollider, ConvexShape, MeshCollider},
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
                        if let Some(contact) = convex_mesh_contact(
                            entity_a,
                            convex_a,
                            transform_a,
                            velocity_a,
                            entity_b,
                            mesh_b,
                            transform_b,
                            &render_resources,
                        ) {
                            contacts.push(contact);
                        }
                        continue;
                    }

                    if let (Some(mesh_a), Some(convex_b)) = (mesh_a, convex_b) {
                        if let Some(contact) = convex_mesh_contact(
                            entity_b,
                            convex_b,
                            transform_b,
                            velocity_b,
                            entity_a,
                            mesh_a,
                            transform_a,
                            &render_resources,
                        ) {
                            contacts.push(contact);
                        }
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
) -> Option<Contact> {
    let render_body = render_resources
        .render_body_manager
        .get_render_body(mesh_collider.render_body_id)?;

    let convex_world = convex_transform.to_mat4();
    let mesh_entity_world = mesh_transform.to_mat4();

    let convex_aabb_world = convex_collider.aabb(&convex_world);
    for part in &render_body.parts {
        let mesh = render_resources.mesh_manager.get_mesh(part.mesh_id)?;
        let bvh = mesh.bvh.as_ref()?;

        let mesh_world = mesh_entity_world * part.local_transform;
        let mesh_world_inv = mesh_world.try_inverse()?;
        if let Some(contact) = convex_mesh_contact_at_transform(
            convex_entity,
            mesh_entity,
            convex_collider,
            convex_world,
            &mesh_world,
            &mesh_world_inv,
            bvh,
        ) {
            return Some(contact);
        }

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
                    if let Some(contact) = convex_mesh_contact_at_transform(
                        convex_entity,
                        mesh_entity,
                        convex_collider,
                        swept_world,
                        &mesh_world,
                        &mesh_world_inv,
                        bvh,
                    ) {
                        return Some(contact);
                    }
                }
            }
        }
    }

    None
}

fn convex_mesh_contact_at_transform(
    convex_entity: Entity,
    mesh_entity: Entity,
    convex_collider: &ConvexCollider,
    convex_world: Mat4,
    mesh_world: &Mat4,
    mesh_world_inv: &Mat4,
    bvh: &crate::collider_component::BVHNode,
) -> Option<Contact> {
    let collider_in_mesh_space = *mesh_world_inv * convex_world;
    let mut hits = Vec::new();
    bvh.query_collider(convex_collider, &collider_in_mesh_space, &mut hits);
    if hits.is_empty() {
        return None;
    }

    let mut best_contact: Option<Contact> = None;
    let mut best_penetration = 0.0;
    let convex_center_world = convex_world.transform_point3(Vec3::ZERO);
    for (tri, hit) in hits {
        let mut normal_world = mesh_world.transform_vector3(hit.normal);
        let normal_len = normal_world.length();
        if normal_len <= f32::EPSILON {
            continue;
        }
        let penetration = hit.penetration * normal_len;
        normal_world /= normal_len;

        let tri_point_world = mesh_world.transform_point3(tri.v0);
        if (convex_center_world - tri_point_world).dot(normal_world) > 0.0 {
            normal_world = -normal_world;
        }

        let contact = Contact {
            entity_a: convex_entity,
            entity_b: mesh_entity,
            normal: normal_world,
            penetration,
        };

        if contact.penetration > best_penetration {
            best_penetration = contact.penetration;
            best_contact = Some(contact);
        }
    }

    best_contact
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
