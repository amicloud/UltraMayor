use bevy_ecs::prelude::*;

use crate::{audio::audio_queue::{AudioInstance, AudioQueue}, components::audio_source_component::AudioSourceComponent};

pub struct AudioSystem;

impl AudioSystem {
    pub fn build_audio_queue(
        query: Query<&AudioSourceComponent, Added<AudioSourceComponent>>,
        mut queue: ResMut<AudioQueue>,
    ) {
        for source in query.iter() {
            queue.instances.push(AudioInstance {
                sound: source.sound.clone(),
                volume: source.volume,
                looping: source.looping,
            });
        }
    }
}
