use bevy_ecs::prelude::*;

use crate::{
    components::physics_event_listener_component::PhysicsEventListenerComponent,
    physics::{
        collision_system::ordered_pair,
        physics_event::{PhysicsEvent, PhysicsEventInfo, PhysicsEventType},
        physics_resource::CollisionFrameData,
    },
};

pub struct PhysicsEventDispatcher;

impl PhysicsEventDispatcher {}

/// We only dispatch collision events for entities that have a PhysicsEventListenerComponent
/// with listen=true to avoid unnecessary event generation for entities that don't care about physics
/// events. This will help optimize performance by reducing the number of events we need to create and process each frame.
pub fn dispatch_physics_events(
    query: Query<&PhysicsEventListenerComponent>,
    mut commands: Commands,
    collision_frame_data: Res<CollisionFrameData>,
) {
    for ((entity_a, entity_b), manifold) in &collision_frame_data.manifolds {
        let pair = ordered_pair(*entity_a, *entity_b);
        let event_type = if collision_frame_data.previous_manifolds.contains_key(&pair) {
            PhysicsEventType::Stay
        } else {
            PhysicsEventType::Hit
        };

        if query.get(*entity_a).is_ok() {
            let event_a = PhysicsEvent {
                entity: *entity_a,
                event_type,
                collision_info: PhysicsEventInfo {
                    normal: manifold.normal,
                    contacts: manifold.contacts.clone(),
                },
                other_entity: *entity_b,
            };
            commands.trigger(event_a);
        }

        if query.get(*entity_b).is_ok() {
            let event_b = PhysicsEvent {
                entity: *entity_b,
                event_type,
                collision_info: PhysicsEventInfo {
                    normal: -manifold.normal,
                    contacts: manifold.contacts.clone(),
                },
                other_entity: *entity_a,
            };
            commands.trigger(event_b);
        }
    }
}
