use bevy_ecs::{
    entity::Entity,
    system::{Query, ResMut},
};

use crate::{TransformComponent, collider_component::ColliderComponent, physics_resource::PhysicsResource};

#[derive(Default)]
pub struct CollisionSystem {}

impl CollisionSystem {
    pub fn do_aabb_collisions(
        query: Query<(Entity, &ColliderComponent, &TransformComponent)>,
        mut phys: ResMut<PhysicsResource>,
    ) {
        let mut colliders: Vec<(Entity, &ColliderComponent)> = Vec::new();
        for (entity, collider, _transform) in &query {
            colliders.push((entity, collider));
        }

        for i in 0..colliders.len() {
            for j in (i + 1)..colliders.len() {
                let (entity_a, collider_a) = colliders[i];
                let (entity_b, collider_b) = colliders[j];

                if collider_a.intersects(collider_b) {
                    println!(
                        "Collision detected between Entity {:?} and Entity {:?}",
                        entity_a, entity_b
                    );
                }
            }
        }
    }
}
