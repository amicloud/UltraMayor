use bevy_ecs::prelude::*;

use crate::{
    TransformComponent, audio::{command_queue::{self, AudioCommand}},
    components::single_audio_listener_component::SingleAudioListenerComponent,
};

pub struct SpatialAudioSystem;


/// This system is responsible for updating the position and rotation of the audio listener
/// and the position of audio source entities in the world.
impl SpatialAudioSystem {
    /// This currently only supports having a single listener
    pub fn update_listener_position(
        query: Query<(Entity, &TransformComponent, &SingleAudioListenerComponent),
            Changed<TransformComponent>>,
        mut audio_command_queue: ResMut<command_queue::AudioCommandQueue>,
    ) {
        if let Some((_, transform, _)) = query.iter().nth(0) {
            audio_command_queue.push(AudioCommand::UpdateListenerInfo {
                listener_info: Some((transform.position, transform.rotation)),
            });
        }
    }
}
