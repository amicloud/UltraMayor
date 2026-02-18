use bevy_ecs::prelude::*;

use crate::physics::{
    collision_event::{CollisionEvent, CollisionEventType, CollisionInfo},
    physics_resource::CollisionFrameData,
};

pub struct CollisionEventDispatcher;

impl CollisionEventDispatcher {
    
}
pub fn dispatch_collision_events(
        mut commands: Commands,
        collision_frame_data: Res<CollisionFrameData>,
    ) {
        for (entity_a, entity_b) in &collision_frame_data.candidate_pairs {
            let contact = collision_frame_data.contacts.iter().find(|c| {
                (c.entity_a == *entity_a && c.entity_b == *entity_b)
                    || (c.entity_a == *entity_b && c.entity_b == *entity_a)
            });

            if let Some(contact) = contact {
                let event_type = if contact.penetration > 0.0 {
                    CollisionEventType::Enter
                } else {
                    CollisionEventType::Exit
                };

                let collision_event_a = CollisionEvent {
                    entity: contact.entity_a,
                    event_type: event_type,
                    collision_info: CollisionInfo {
                        normal: contact.normal,
                        penetration: contact.penetration,
                        contact_point: contact.contact_point,
                    },
                    other_entity: contact.entity_b,
                };

                let collision_event_b = CollisionEvent {
                    entity: contact.entity_b,
                    event_type,
                    collision_info: CollisionInfo {
                        normal: -contact.normal, // Invert normal for the other entity
                        penetration: contact.penetration,
                        contact_point: contact.contact_point,
                    },
                    other_entity: contact.entity_a,
                };

                commands.trigger(collision_event_a);
                commands.trigger(collision_event_b);
            }
        }
    }