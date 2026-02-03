use bevy_ecs::prelude::{Changed, Entity, Query, Res, ResMut};
use glam::{Mat4, Vec3};
use std::collections::HashMap;

use crate::{
    TransformComponent,
    collider_component::{BoxCollider, MeshCollider},
    mesh::AABB,
    physics_component::{PhysicsComponent, PhysicsType},
    physics_resource::{Contact, Impulse, PhysicsResource},
    render_resource_manager::RenderResourceManager,
    velocity_component::VelocityComponent,
};

#[derive(Default)]
pub struct CollisionSystem {}

impl CollisionSystem {
    pub fn update_world_aabb_cache(
        query: Query<
            (
                Entity,
                &TransformComponent,
                Option<&BoxCollider>,
                Option<&MeshCollider>,
            ),
            Changed<TransformComponent>,
        >,
        render_resources: Res<RenderResourceManager>,
        mut phys: ResMut<PhysicsResource>,
    ) {
        for (entity, transform, box_collider, mesh_collider) in &query {
            if let Some(mesh_collider) = mesh_collider {
                if let Some(local_aabb) = render_body_local_aabb(
                    mesh_collider.render_body_id,
                    &render_resources,
                ) {
                    let world_aabb = transform_aabb(local_aabb, transform);
                    phys.world_aabbs.insert(entity, world_aabb);
                    continue;
                }
            }

            if let Some(box_collider) = box_collider {
                let world_aabb = transform_aabb(box_collider.aabb, transform);
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
                Option<&BoxCollider>,
                Option<&MeshCollider>,
            ),
            Changed<TransformComponent>,
        >,
        all_query: Query<(
            Entity,
            &TransformComponent,
            Option<&VelocityComponent>,
            Option<&BoxCollider>,
            Option<&MeshCollider>,
        )>,
        render_resources: Res<RenderResourceManager>,
        mut phys: ResMut<PhysicsResource>,
    ) {
        // Collect moving entities
        let moving_entities: Vec<Entity> =
            moving_query.iter().map(|(e, _, _, _, _)| e).collect();
        let mut contacts = Vec::new();

        // Iterate over moving entities only
        for &entity_a in &moving_entities {
            let Some(aabb_a) = phys.world_aabbs.get(&entity_a) else {
                continue;
            };

            let Ok((_, transform_a, velocity_a, box_a, mesh_a)) = moving_query.get(entity_a) else {
                continue;
            };

            // Compare against all colliders (moving + static)
            for (entity_b, transform_b, velocity_b, box_b, mesh_b) in &all_query {
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

                if aabb_intersects(&aabb_a_swept, aabb_b) {
                    if box_a.is_some() && box_b.is_some() {
                        if let Some(contact) = box_box_contact(entity_a, aabb_a, entity_b, aabb_b)
                        {
                            contacts.push(contact);
                        }
                        continue;
                    }

                    if let (Some(box_a), Some(mesh_b)) = (box_a, mesh_b) {
                        if let Some(contact) = box_mesh_contact(
                            entity_a,
                            box_a,
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

                    if let (Some(mesh_a), Some(box_b)) = (mesh_a, box_b) {
                        if let Some(contact) = box_mesh_contact(
                            entity_b,
                            box_b,
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

    pub fn resolve_contacts(
        query: Query<(Option<&VelocityComponent>, Option<&PhysicsComponent>)>,
        mut phys: ResMut<PhysicsResource>,
        mut transforms: Query<&mut TransformComponent>,
    ) {
        let mut impulses = Vec::new();
        let mut corrections: HashMap<Entity, Vec3> = HashMap::new();
        println!("Number of contacts to resolve: {}", phys.contacts.len());
        for contact in phys.contacts.iter() {
            let (vel_a, phys_a) = query
                .get(contact.entity_a)
                .map(|(v, p)| (v.map(|v| v.translational), p))
                .unwrap_or((None, None));
            let (vel_b, phys_b) = query
                .get(contact.entity_b)
                .map(|(v, p)| (v.map(|v| v.translational), p))
                .unwrap_or((None, None));

            let velocity_a = vel_a.unwrap_or(Vec3::ZERO);
            let velocity_b = vel_b.unwrap_or(Vec3::ZERO);

            let inv_mass_a = phys_a
                .filter(|p| matches!(p.physics_type, PhysicsType::Dynamic))
                .map(|p| if p.mass > 0.0 { 1.0 / p.mass } else { 0.0 })
                .unwrap_or(0.0);
            let inv_mass_b = phys_b
                .filter(|p| matches!(p.physics_type, PhysicsType::Dynamic))
                .map(|p| if p.mass > 0.0 { 1.0 / p.mass } else { 0.0 })
                .unwrap_or(0.0);
            let inv_mass_sum = inv_mass_a + inv_mass_b;

            if inv_mass_sum == 0.0 {
                println!("Both entities have infinite mass, skipping contact resolution.");
                continue;
            }

            let restitution = match (phys_a, phys_b) {
                (Some(a), Some(b)) => a.restitution.min(b.restitution),
                (Some(a), None) => a.restitution,
                (None, Some(b)) => b.restitution,
                (None, None) => 0.0,
            };

            let relative_velocity = velocity_b - velocity_a;
            let vel_along_normal = relative_velocity.dot(contact.normal);

            let penetration_slop = 0.01;
            let resting_threshold = 0.2;
            if vel_along_normal.abs() < resting_threshold
                && contact.penetration <= penetration_slop
            {
                println!("Resting contact detected, skipping impulse resolution.");
                continue;
            }
            let effective_restitution = if vel_along_normal.abs() < resting_threshold {
                0.0
            } else {
                restitution
            };

            let mut normal_impulse = 0.0;
            if vel_along_normal < -resting_threshold {
                normal_impulse = (-(1.0 + effective_restitution) * vel_along_normal) / inv_mass_sum;
                let impulse = contact.normal * normal_impulse;
                println!("Applying normal impulse: {:?}", impulse);
                impulses.push(Impulse {
                    entity: contact.entity_a,
                    linear: -impulse,
                    angular: Vec3::ZERO,
                });
                impulses.push(Impulse {
                    entity: contact.entity_b,
                    linear: impulse,
                    angular: Vec3::ZERO,
                });
            } else {
                println!("No normal impulse applied for non-colliding contact.");
            }

            if normal_impulse > 0.0 {
                let tangent = relative_velocity - vel_along_normal * contact.normal;
                let tangent_len = tangent.length();
                if tangent_len > 1e-6 {
                    let tangent_dir = tangent / tangent_len;
                    let jt = -relative_velocity.dot(tangent_dir) / inv_mass_sum;

                    let friction = match (phys_a, phys_b) {
                        (Some(a), Some(b)) => (a.friction * b.friction).sqrt(),
                        (Some(a), None) => a.friction,
                        (None, Some(b)) => b.friction,
                        (None, None) => 0.0,
                    };

                    let max_friction = friction * normal_impulse;
                    let friction_impulse = jt.clamp(-max_friction, max_friction);
                    let impulse = tangent_dir * friction_impulse;
                    println!("Applying friction impulse: {:?}", impulse);
                    impulses.push(Impulse {
                        entity: contact.entity_a,
                        linear: -impulse,
                        angular: Vec3::ZERO,
                    });
                    impulses.push(Impulse {
                        entity: contact.entity_b,
                        linear: impulse,
                        angular: Vec3::ZERO,
                    });
                }
            } else {
                println!("No normal impulse applied, skipping friction resolution.");
            }

            let correction_percent = 0.2;
            let penetration = (contact.penetration - penetration_slop).max(0.0);
            if penetration > 0.0 {
                println!("Applying positional correction for penetration: {}", penetration);
                let correction =
                    contact.normal * (penetration * correction_percent / inv_mass_sum);

                if inv_mass_a > 0.0 {
                    let entry = corrections.entry(contact.entity_a).or_insert(Vec3::ZERO);
                    *entry -= correction * inv_mass_a;
                }
                if inv_mass_b > 0.0 {
                    let entry = corrections.entry(contact.entity_b).or_insert(Vec3::ZERO);
                    *entry += correction * inv_mass_b;
                }
            }
        }
        for impulse in impulses {
            println!("Adding impulse...");
            phys.add_impulse(impulse.entity, impulse.linear, impulse.angular);
        }
        for (entity, correction) in corrections {
            if let Ok(mut transform) = transforms.get_mut(entity) {
                transform.position += correction;
            }
        }
        phys.contacts.clear();
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

fn box_box_contact(
    entity_a: Entity,
    aabb_a: &AABB,
    entity_b: Entity,
    aabb_b: &AABB,
) -> Option<Contact> {
    let delta = (aabb_b.min + aabb_b.max) * 0.5 - (aabb_a.min + aabb_a.max) * 0.5;
    let overlap_x = (aabb_a.max.x - aabb_a.min.x + aabb_b.max.x - aabb_b.min.x) * 0.5
        - delta.x.abs();
    let overlap_y = (aabb_a.max.y - aabb_a.min.y + aabb_b.max.y - aabb_b.min.y) * 0.5
        - delta.y.abs();
    let overlap_z = (aabb_a.max.z - aabb_a.min.z + aabb_b.max.z - aabb_b.min.z) * 0.5
        - delta.z.abs();

    let (penetration, normal) = if overlap_x < overlap_y && overlap_x < overlap_z {
        (overlap_x, Vec3::new(delta.x.signum(), 0.0, 0.0))
    } else if overlap_y < overlap_z {
        (overlap_y, Vec3::new(0.0, delta.y.signum(), 0.0))
    } else {
        (overlap_z, Vec3::new(0.0, 0.0, delta.z.signum()))
    };

    Some(Contact {
        entity_a,
        entity_b,
        normal,
        penetration,
    })
}

fn box_mesh_contact(
    box_entity: Entity,
    box_collider: &BoxCollider,
    box_transform: &TransformComponent,
    box_velocity: Option<&VelocityComponent>,
    mesh_entity: Entity,
    mesh_collider: &MeshCollider,
    mesh_transform: &TransformComponent,
    render_resources: &RenderResourceManager,
) -> Option<Contact> {
    let render_body = render_resources
        .render_body_manager
        .get_render_body(mesh_collider.render_body_id)?;

    let box_aabb_world = transform_aabb(box_collider.aabb, box_transform);
    let box_world = box_transform.to_mat4();
    let mesh_entity_world = mesh_transform.to_mat4();

    for part in &render_body.parts {
        let mesh = render_resources.mesh_manager.get_mesh(part.mesh_id)?;
        let bvh = mesh.bvh.as_ref()?;

        let mesh_world = mesh_entity_world * part.local_transform;
        let mesh_world_inv = mesh_world.try_inverse()?;
        if let Some(contact) = box_mesh_contact_at_transform(
            box_entity,
            mesh_entity,
            box_collider,
            box_transform,
            box_world,
            &mesh_world,
            &mesh_world_inv,
            bvh,
        ) {
            return Some(contact);
        }

        if let Some(velocity) = box_velocity {
            let delta = velocity.translational * fixed_dt();
            let distance = delta.length();
            if distance > 0.0 {
                let step = (box_aabb_world.max - box_aabb_world.min)
                    .abs()
                    .min_element()
                    .max(0.01)
                    * 0.5;
                let steps = ((distance / step).ceil() as i32).clamp(1, 20);

                for i in 1..=steps {
                    let t = i as f32 / steps as f32;
                    let swept_position = box_transform.position + delta * t;
                    let swept_transform = TransformComponent {
                        position: swept_position,
                        rotation: box_transform.rotation,
                        scale: box_transform.scale,
                    };
                    let swept_world = swept_transform.to_mat4();
                    if let Some(contact) = box_mesh_contact_at_transform(
                        box_entity,
                        mesh_entity,
                        box_collider,
                        &swept_transform,
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

fn box_mesh_contact_at_transform(
    box_entity: Entity,
    mesh_entity: Entity,
    box_collider: &BoxCollider,
    box_transform: &TransformComponent,
    box_world: Mat4,
    mesh_world: &Mat4,
    mesh_world_inv: &Mat4,
    bvh: &crate::collider_component::BVHNode,
) -> Option<Contact> {
    let box_aabb_world = transform_aabb(box_collider.aabb, box_transform);
    let collider_in_mesh_space = *mesh_world_inv * box_world;
    let mut hits = Vec::new();
    bvh.query_collider(box_collider, &collider_in_mesh_space, &mut hits);
    if hits.is_empty() {
        return None;
    }

    let (tri, hit) = &hits[0];
    let mut normal_world = mesh_world.transform_vector3(hit.normal);
    let normal_len = normal_world.length();
    if normal_len <= f32::EPSILON {
        return None;
    }
    let penetration = hit.penetration * normal_len;
    normal_world /= normal_len;

    let box_center_world = (box_aabb_world.min + box_aabb_world.max) * 0.5;
    let tri_point_world = mesh_world.transform_point3(tri.v0);
    if (box_center_world - tri_point_world).dot(normal_world) > 0.0 {
        normal_world = -normal_world;
    }

    let contact = Contact {
        entity_a: box_entity,
        entity_b: mesh_entity,
        normal: normal_world,
        penetration,
    };
    println!(
        "Box-Mesh collision detected: Box Entity {:?}, Mesh Entity {:?}, Normal {:?}, Penetration {}",
        box_entity, mesh_entity, contact.normal, contact.penetration
    );
    Some(contact)
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
