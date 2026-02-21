use std::collections::HashMap;

use bevy_ecs::prelude::*;
use slotmap::SlotMap;

use crate::assets::{handles::SoundHandle, sound::Sound};

#[derive(Resource, Default)]
pub struct SoundResource {
    pub sounds: SlotMap<SoundHandle, Sound>,
    pub name_map: HashMap<String, SoundHandle>,
}

impl SoundResource {
    pub fn add_sound(&mut self, sound: Sound, name: String) -> SoundHandle {
        let handle = self.sounds.insert(sound);
        dbg!("Added sound with ID: {:?} and name: {}", handle, &name);
        self.name_map.insert(name, handle);
        handle
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

    pub fn get_by_name(&self, name: &str) -> Option<SoundHandle> {
        self.name_map.get(name).copied()
    }
}
