use bevy_ecs::prelude::*;
use engine::TransformComponent;
use glam::Quat;

#[derive(Component, Debug, Clone, Copy)]
pub struct BowlFloatComponent {
    pub base_height: f32,
    pub amplitude: f32,
    pub speed: f32,
}

#[derive(Resource, Debug, Default)]
pub struct BowlFloatTime {
    pub seconds: f32,
}

pub fn update_bowl_float(
    mut time: ResMut<BowlFloatTime>,
    mut query: Query<(&mut TransformComponent, &BowlFloatComponent)>,
) {
    const FIXED_DT: f32 = 1.0 / 60.0;
    time.seconds += FIXED_DT;

    for (mut transform, float) in &mut query {
        let offset = (time.seconds * float.speed).sin() * float.amplitude;
        transform.position.z = float.base_height + offset;
        transform.rotation = Quat::from_rotation_x((time.seconds * float.speed).cos() * float.amplitude.to_radians());
    }
}
