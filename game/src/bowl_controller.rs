// use bevy_ecs::prelude::*;
// use engine::{TimeResource, VelocityComponent};

// #[derive(Component, Debug, Clone, Copy)]
// pub struct BowlFloatComponent {
//     pub base_height: f32,
//     pub amplitude: f32,
//     pub speed: f32,
// }
// pub fn update_bowl_float(
//     mut time: ResMut<TimeResource>,
//     mut query: Query<(&BowlFloatComponent, &mut VelocityComponent)>,

// ) {
//     for (bowl, mut velocity) in &mut query {
//         let offset = triangle_wave(time.total_time() as f32 * bowl.speed, 1.0) * bowl.amplitude;
//         velocity.translational.z = bowl.base_height + offset;
//     }
// }

// fn triangle_wave(time: f32, period: f32) -> f32 {
//     let t = time % period;
//     if t < period / 2.0 {
//         (t / (period / 2.0)) * 2.0 - 1.0
//     } else {
//         ((period - t) / (period / 2.0)) * 2.0 - 1.0
//     }
// }
