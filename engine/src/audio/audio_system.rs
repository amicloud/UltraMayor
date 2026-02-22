use bevy_ecs::prelude::*;

use crate::{
    audio::command_queue::{AudioCommand, AudioCommandQueue},
    components::audio_source_component::AudioSourceComponent,
};

pub struct AudioSystem;

impl AudioSystem {
    pub fn build_command_queue(
        query: Query<&AudioSourceComponent, Added<AudioSourceComponent>>,
        mut queue: ResMut<AudioCommandQueue>,
    ) {
        queue.clear();
        for source in query.iter() {
            queue.push(AudioCommand::PlaySound {
                track: 0,
                sound: source.sound,
                volume: source.volume,
                looping: source.looping,
                location: None,
            });
        }
    }
}
