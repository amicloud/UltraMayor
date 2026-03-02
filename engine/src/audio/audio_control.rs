use bevy_ecs::prelude::*;
use glam::{Quat, Vec3};

use crate::{
    SoundHandle,
    audio::audio_mixer::{ListenerInfo, SourceInfo},
};

#[derive(Debug)]
pub(crate) enum AudioCommand {
    SpawnSpatialEmitter {
        track: u8,
        sound: SoundHandle,
        volume: f32,
        looping: bool,
        source: Entity,
    },
    PlayOneShotAtLocation {
        track: u8,
        sound: SoundHandle,
        volume: f32,
        location: Vec3,
    },
    PlayOneShot {
        track: u8,
        sound: SoundHandle,
        volume: f32,
    },
    MuteTrack {
        track: u8,
    },
    UnmuteTrack {
        track: u8,
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
    RemoveSourceInfo {
        entity: Entity,
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
        track: u8,
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

    pub(crate) fn remove_spatial_emitter(&mut self, source: Entity) {
        self.push(AudioCommand::RemoveSourceInfo { entity: source });
    }

    pub fn play_one_shot(&mut self, track: u8, sound: SoundHandle, volume: f32) {
        self.push(AudioCommand::PlayOneShot {
            track,
            sound,
            volume,
        });
    }

    pub fn play_one_shot_at_location(
        &mut self,
        track: u8,
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

    pub fn mute_track(&mut self, track: u8) {
        self.push(AudioCommand::MuteTrack { track });
    }

    pub fn unmute_track(&mut self, track: u8) {
        self.push(AudioCommand::UnmuteTrack { track });
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
