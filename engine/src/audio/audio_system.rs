use bevy_ecs::prelude::*;

use crate::{
    TransformComponent, audio::command_queue::{AudioCommand, AudioCommandQueue}, components::audio_source_component::AudioSourceComponent
};

pub struct AudioSystem;

impl AudioSystem {
    pub fn build_command_queue(
        query: Query<(Entity, &AudioSourceComponent, Option<&TransformComponent>),  Added<AudioSourceComponent>>,
        mut queue: ResMut<AudioCommandQueue>,
    ) {
        for (entity, source, transform) in query.iter() {
            queue.push(AudioCommand::PlaySound {
                track: 0,
                sound: source.sound,
                volume: source.volume,
                looping: source.looping,
                location: transform.map(|t| t.position),
                source: Some(entity),
            });
        }
    }

    pub fn clear_command_queue(mut queue: ResMut<AudioCommandQueue>) {
        queue.clear();
    }
}
