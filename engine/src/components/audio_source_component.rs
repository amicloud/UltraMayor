use bevy_ecs::prelude::*;

use crate::SoundHandle;

#[derive(Component)]
pub struct AudioSourceComponent {
    pub sound: SoundHandle,
    pub volume: f32,
    pub pitch: f32,
    pub looping: bool,
    pub spatial: bool,
}
