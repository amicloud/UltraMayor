use bevy_ecs::prelude::*;

use crate::SoundHandle;



pub struct AudioInstance {
    pub sound: SoundHandle,
    pub volume: f32,
    pub looping: bool,
}


#[derive(Resource, Default)]
pub struct AudioQueue {
    pub instances: Vec<AudioInstance>,
}