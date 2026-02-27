use bevy_ecs::prelude::*;
use glam::Vec3;

use crate::{
    TransformComponent,
    audio::audio_control::{AudioControl},
    components::simple_on_hit_audio_component::SimpleOnHitAudioComponent,
    physics::{
        collision_system::ordered_pair, physics_event::PhysicsEventType,
        physics_resource::CollisionFrameData,
    },
};

pub struct SimplePhysAudioSystem {}

impl SimplePhysAudioSystem {
    pub fn on_hit_audio_system(
        query: Query<(&SimpleOnHitAudioComponent, &TransformComponent)>,
        collision_frame_data: Res<CollisionFrameData>,
        mut audio_control: ResMut<AudioControl>,
    ) {
        for manifold_entry in collision_frame_data.manifolds.iter() {
            let pair = ordered_pair(manifold_entry.entity_a, manifold_entry.entity_b);
            let event_type = if collision_frame_data.previous_manifolds.get(pair).is_some() {
                PhysicsEventType::Stay
            } else {
                PhysicsEventType::Hit
            };

            match event_type {
                PhysicsEventType::Hit => {
                    if query.get(manifold_entry.entity_a).is_ok() {
                        let volume = query.get(manifold_entry.entity_a).unwrap().0.volume;
                        let force_modifier = query
                            .get(manifold_entry.entity_a)
                            .unwrap()
                            .0
                            .force_volume_scale;
                        let contact_force = manifold_entry
                            .manifold
                            .contacts
                            .iter()
                            .map(|c| c.penetration)
                            .sum::<f32>();
                        let final_volume =
                            volume * (contact_force * force_modifier).clamp(0.0, 1.0);
                        dbg!(final_volume);
                        let sound_position = manifold_entry
                            .manifold
                            .contacts
                            .iter()
                            .map(|c| c.contact_point)
                            .sum::<Vec3>()
                            / manifold_entry.manifold.contacts.len() as f32;
                        audio_control.play_one_shot_at_location(
                            0,
                            query.get(manifold_entry.entity_a).unwrap().0.sound_handle,
                            final_volume,
                            sound_position,
                        );
                    }

                    if query.get(manifold_entry.entity_b).is_ok() {
                        let volume = query.get(manifold_entry.entity_b).unwrap().0.volume;
                        let force_modifier = query
                            .get(manifold_entry.entity_b)
                            .unwrap()
                            .0
                            .force_volume_scale;
                        let contact_force = manifold_entry
                            .manifold
                            .contacts
                            .iter()
                            .map(|c| c.penetration)
                            .sum::<f32>();
                        let final_volume =
                            volume * (contact_force * force_modifier).clamp(0.0, 1.0);
                        dbg!(final_volume);
                        let sound_position = manifold_entry
                            .manifold
                            .contacts
                            .iter()
                            .map(|c| c.contact_point)
                            .sum::<Vec3>()
                            / manifold_entry.manifold.contacts.len() as f32;
                        audio_control.play_one_shot_at_location(
                            0,
                            query.get(manifold_entry.entity_b).unwrap().0.sound_handle,
                            final_volume,
                            sound_position,
                        );
                    }
                }
                PhysicsEventType::Stay => continue,
            }
        }
    }
}
