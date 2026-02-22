use bevy_ecs::prelude::*;

use crate::{
    TransformComponent, audio::{command_queue::{self, AudioCommand}},
    components::single_audio_listener_component::SingleAudioListenerComponent,
};

pub struct SpatialAudioSystem;

impl SpatialAudioSystem {
    /// This currently only supports having a single listener
    pub fn update_listener_position(
        query: Query<(Entity, &TransformComponent, &SingleAudioListenerComponent)>,
        mut audio_command_queue: ResMut<command_queue::AudioCommandQueue>,
    ) {
        if let Some((_, transform, _)) = query.iter().nth(0) {
            audio_command_queue.push(AudioCommand::UpdateListenerPosition {
                position: transform.position,
            });
        }
    }
}
