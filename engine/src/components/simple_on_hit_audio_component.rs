use bevy_ecs::prelude::*;

use crate::SoundHandle;

#[derive(Component)]
pub struct SimpleOnHitAudioComponent {
    pub sound_handle: SoundHandle,
    pub volume: f32,
    pub pitch: f32,
    pub force_volume_scale: f32,
}
