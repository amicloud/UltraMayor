use bevy_ecs::prelude::*;
use slotmap::SlotMap;

use crate::{
    assets::{handles::SoundHandle, sound::Sound},
};

#[derive(Resource, Default)]
pub struct SoundResource {
    pub sounds: SlotMap<SoundHandle, Sound>,
}

impl SoundResource {
    pub fn add_sound(&mut self, sound: Sound) -> SoundHandle {
        self.sounds.insert(sound)
    }

    pub fn get_sound(&self, sound_id: SoundHandle) -> Option<&Sound> {
        self.sounds.get(sound_id)
    }

    pub fn get_sound_mut(&mut self, sound_id: SoundHandle) -> Option<&mut Sound> {
        self.sounds.get_mut(sound_id)
    }

    #[allow(dead_code)]
    pub fn remove_sound(&mut self, sound_id: SoundHandle) {
        if self.sounds.remove(sound_id).is_some() {
            println!("Removed sound with ID: {:?}", sound_id);
        } else {
            println!("Sound with ID {:?} not found for removal", sound_id);
        }
    }
}
