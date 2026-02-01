use bevy_ecs::prelude::{Changed, Entity, Query, Res, ResMut, With};
use glam::Vec3;

use crate::{
    collider_component::ColliderComponent, mesh::AABB, physics_resource::PhysicsResource,
    TransformComponent,
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

    pub fn do_aabb_collisions(
    moving_query: Query<(Entity, &ColliderComponent, &TransformComponent), Changed<TransformComponent>>,
    all_query: Query<(Entity, &ColliderComponent), With<ColliderComponent>>,
    phys: Res<PhysicsResource>,
) {
    // Collect moving entities
    let moving_entities: Vec<Entity> = moving_query.iter().map(|(e, _, _)| e).collect();
    
    // Iterate over moving entities only
    for &entity_a in &moving_entities {
        let Some(aabb_a) = phys.world_aabbs.get(&entity_a) else { continue; };

        // Compare against all colliders (moving + static)
        for (entity_b, _collider_b) in &all_query {
            if entity_a == entity_b { continue; } // skip self

            let Some(aabb_b) = phys.world_aabbs.get(&entity_b) else { continue; };

            if aabb_intersects(aabb_a, aabb_b) {
                println!(
                    "Collision detected between Entity {:?} and Entity {:?}",
                    entity_a, entity_b
                );
            }
        }
    }
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
