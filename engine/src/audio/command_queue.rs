use bevy_ecs::prelude::*;
use glam::Vec3;

use crate::{SoundHandle, audio::audio_mixer::ListenerInfo};

#[derive(Debug)]
pub enum AudioCommand {
    PlaySound {
        track: usize,
        sound: SoundHandle,
        volume: f32,
        looping: bool,
        location: Option<Vec3>,
    },
    PauseTrack {
        track: usize,
    },
    ResumeTrack {
        track: usize,
    },
    PauseMix,
    ResumeMix,
    MuteMix,
    UnmuteMix,
    UpdateListenerInfo {
        listener_info: ListenerInfo,
    },
}

#[derive(Resource, Default)]
pub struct AudioCommandQueue {
    pub queue: Vec<AudioCommand>,
}

impl AudioCommandQueue {
    pub fn clear(&mut self) {
        self.queue.clear();
    }

    pub fn push(&mut self, command: AudioCommand) {
        self.queue.push(command);
    }

    pub fn pop(&mut self) -> Option<AudioCommand> {
        self.queue.pop()
    }
}
