use bevy_ecs::prelude::{Changed, Entity, Query, Res, ResMut, With};
use glam::Vec3;

use crate::{
    TransformComponent, collider_component::ColliderComponent, mesh::AABB, physics_resource::{Contact, Impulse, PhysicsResource}
};

#[derive(Default)]
pub struct CollisionSystem {}

impl CollisionSystem {
    pub fn update_world_aabb_cache(
        query: Query<
            (Entity, &ColliderComponent, &TransformComponent),
            Changed<TransformComponent>,
        >,
        mut phys: ResMut<PhysicsResource>,
    ) {
        for (entity, collider, transform) in &query {
            let world_aabb = transform_aabb(collider.aabb(), transform);
            phys.world_aabbs.insert(entity, world_aabb);
        }
    }

    pub fn generate_contacts(
        moving_query: Query<
            (Entity, &ColliderComponent, &TransformComponent),
            Changed<TransformComponent>,
        >,
        all_query: Query<(Entity, &ColliderComponent), With<ColliderComponent>>,
        mut phys: ResMut<PhysicsResource>,
    ) {
        // Collect moving entities
        let moving_entities: Vec<Entity> = moving_query.iter().map(|(e, _, _)| e).collect();
        let mut contacts = Vec::new();

        // Iterate over moving entities only
        for &entity_a in &moving_entities {
            let Some(aabb_a) = phys.world_aabbs.get(&entity_a) else {
                continue;
            };

            // Compare against all colliders (moving + static)
            for (entity_b, _collider_b) in &all_query {
                if entity_a == entity_b {
                    continue;
                } // skip self

                let Some(aabb_b) = phys.world_aabbs.get(&entity_b) else {
                    continue;
                };

                if aabb_intersects(aabb_a, aabb_b) {
                    println!(
                        "Contact detected between Entity {:?} and Entity {:?}",
                        entity_a, entity_b
                    );
                    // Simple approximation: normal along the largest penetration axis
                    let delta = (aabb_b.min + aabb_b.max) * 0.5 - (aabb_a.min + aabb_a.max) * 0.5;
                    let overlap_x = (aabb_a.max.x - aabb_a.min.x + aabb_b.max.x - aabb_b.min.x)
                        * 0.5
                        - delta.x.abs();
                    let overlap_y = (aabb_a.max.y - aabb_a.min.y + aabb_b.max.y - aabb_b.min.y)
                        * 0.5
                        - delta.y.abs();
                    let overlap_z = (aabb_a.max.z - aabb_a.min.z + aabb_b.max.z - aabb_b.min.z)
                        * 0.5
                        - delta.z.abs();

                    let (penetration, normal) = if overlap_x < overlap_y && overlap_x < overlap_z {
                        (overlap_x, Vec3::new(delta.x.signum(), 0.0, 0.0))
                    } else if overlap_y < overlap_z {
                        (overlap_y, Vec3::new(0.0, delta.y.signum(), 0.0))
                    } else {
                        (overlap_z, Vec3::new(0.0, 0.0, delta.z.signum()))
                    };

                    contacts.push(Contact {
                        entity_a,
                        entity_b,
                        normal,
                        penetration,
                    });
                }
            }
        }

        for contact in contacts {
            phys.add_contact(contact);
        }
    }

    pub fn resolve_contacts(
        mut phys: ResMut<PhysicsResource>
    ) {
        let mut impulses = Vec::new();
        for contact in phys.contacts.iter() {
            // Calculate impulse
            let impulse = contact.normal * contact.penetration;
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
        for impulse in impulses {
            phys.add_impulse(impulse.entity, impulse.linear, impulse.angular);
        }
        phys.contacts.clear();
    }
}

fn transform_aabb(local: AABB, transform: &TransformComponent) -> AABB {
    let matrix = transform.to_mat4();
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

    let mut world_min = matrix.transform_point3(corners[0]);
    let mut world_max = world_min;

    for corner in corners.iter().skip(1) {
        let world = matrix.transform_point3(*corner);
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
