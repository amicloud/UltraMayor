use bevy_ecs::prelude::*;

use crate::{
    TransformComponent, audio::audio_control::AudioControl,
    components::audio_source_component::AudioSourceComponent,
};

pub struct AudioCommandQueueSystem;

impl AudioCommandQueueSystem {
    pub fn build_command_queue(
        query: Query<
            (Entity, &AudioSourceComponent, &TransformComponent),
            Added<AudioSourceComponent>,
        >,
        mut audio: ResMut<AudioControl>,
    ) {
        for (entity, source, _) in query.iter() {
            audio.spawn_spatial_emitter(0, source.sound, source.volume, source.looping, entity);
        }
    }

    pub fn clear_command_queue(mut audio: ResMut<AudioControl>) {
        audio.clear();
    }
}
