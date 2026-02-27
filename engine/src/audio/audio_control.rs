use bevy_ecs::prelude::*;
use glam::{Quat, Vec3};

use crate::{
    SoundHandle,
    audio::audio_mixer::{ListenerInfo, SourceInfo},
};

#[derive(Debug)]
pub(crate) enum AudioCommand {
    SpawnSpatialEmitter {
        track: usize,
        sound: SoundHandle,
        volume: f32,
        looping: bool,
        source: Entity,
    },
    PlayOneShotAtLocation {
        track: usize,
        sound: SoundHandle,
        volume: f32,
        location: Vec3,
    },
    PlayOneShot {
        track: usize,
        sound: SoundHandle,
        volume: f32,
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
        info: ListenerInfo,
    },
    UpdateSourceInfo {
        entity: Entity,
        info: SourceInfo,
    },
}

#[derive(Resource, Default)]
pub struct AudioControl {
    queue: Vec<AudioCommand>,
}

impl AudioControl {
    pub(crate) fn clear(&mut self) {
        self.queue.clear();
    }

    fn push(&mut self, command: AudioCommand) {
        self.queue.push(command);
    }

    pub(crate) fn update_listener_info(&mut self, position: Vec3, rotation: Quat) {
        self.push(AudioCommand::UpdateListenerInfo {
            info: (position, rotation),
        });
    }

    pub(crate) fn update_source_info(&mut self, entity: Entity, position: Vec3) {
        self.push(AudioCommand::UpdateSourceInfo {
            entity,
            info: position,
        });
    }

    pub(crate) fn spawn_spatial_emitter(
        &mut self,
        track: usize,
        sound: SoundHandle,
        volume: f32,
        looping: bool,
        source: Entity,
    ) {
        self.push(AudioCommand::SpawnSpatialEmitter {
            track,
            sound,
            volume,
            looping,
            source,
        });
    }

    pub fn play_one_shot(&mut self, track: usize, sound: SoundHandle, volume: f32) {
        self.push(AudioCommand::PlayOneShot {
            track,
            sound,
            volume,
        });
    }

    pub fn play_one_shot_at_location(
        &mut self,
        track: usize,
        sound: SoundHandle,
        volume: f32,
        location: Vec3,
    ) {
        self.push(AudioCommand::PlayOneShotAtLocation {
            track,
            sound,
            volume,
            location,
        });
    }

    pub fn pause_track(&mut self, track: usize) {
        self.push(AudioCommand::PauseTrack { track });
    }

    pub fn resume_track(&mut self, track: usize) {
        self.push(AudioCommand::ResumeTrack { track });
    }

    pub fn pause_mix(&mut self) {
        self.push(AudioCommand::PauseMix);
    }

    pub fn resume_mix(&mut self) {
        self.push(AudioCommand::ResumeMix);
    }

    pub fn mute_mix(&mut self) {
        self.push(AudioCommand::MuteMix);
    }

    pub fn unmute_mix(&mut self) {
        self.push(AudioCommand::UnmuteMix);
    }

    pub(crate) fn queue(&self) -> &[AudioCommand] {
        &self.queue
    }
}
