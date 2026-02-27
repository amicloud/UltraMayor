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
    for manifold_entry in collision_frame_data.manifolds.iter() {
        let pair = ordered_pair(manifold_entry.entity_a, manifold_entry.entity_b);
        let event_type = if collision_frame_data.previous_manifolds.get(pair).is_some() {
            PhysicsEventType::Stay
        } else {
            PhysicsEventType::Hit
        };

        if query.get(manifold_entry.entity_a).is_ok() {
            let event_a = PhysicsEvent {
                entity: manifold_entry.entity_a,
                event_type,
                collision_info: PhysicsEventInfo {
                    normal: manifold_entry.manifold.normal,
                    contacts: manifold_entry.manifold.contacts.clone(),
                    relative_normal_speed: manifold_entry.manifold.relative_normal_speed,
                    impact_impulse: manifold_entry.manifold.impact_impulse,
                    impact_energy: manifold_entry.manifold.impact_energy,
                },
                other_entity: manifold_entry.entity_b,
            };
            commands.trigger(event_a);
        }

        if query.get(manifold_entry.entity_b).is_ok() {
            let event_b = PhysicsEvent {
                entity: manifold_entry.entity_b,
                event_type,
                collision_info: PhysicsEventInfo {
                    normal: -manifold_entry.manifold.normal,
                    contacts: manifold_entry.manifold.contacts.clone(),
                    relative_normal_speed: manifold_entry.manifold.relative_normal_speed,
                    impact_impulse: manifold_entry.manifold.impact_impulse,
                    impact_energy: manifold_entry.manifold.impact_energy,
                },
                other_entity: manifold_entry.entity_a,
            };
            commands.trigger(event_b);
        }
    }
}
