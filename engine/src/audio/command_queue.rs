use bevy_ecs::prelude::*;

use crate::SoundHandle;


pub enum AudioCommand {
    PlaySound {
        track: usize,
        sound: SoundHandle,
        volume: f32,
        looping: bool,
    },
    PauseTrack {
        track: usize,
    },
    ResumeTrack {
        track: usize,
    },
    PauseMix,
    ResumeMix,
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