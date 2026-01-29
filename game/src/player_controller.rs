use bevy_ecs::prelude::*;
use nalgebra::Vector3;
use ultramayor_engine::{TransformComponent, VelocityComponent};

use crate::input_controller::InputState;

/// Simple flying camera movement settings.
#[derive(Component, Debug)]
pub struct PlayerComponent {
    pub speed: f32,
}

/// Applies WASD + Space/Shift to the active camera's velocity.
pub fn apply_player_input(
    input_state: Res<InputState>,
    mut query: Query<(&TransformComponent, &PlayerComponent, &mut VelocityComponent)>,
) {
    for (transform, controller, mut velocity) in &mut query {
        let forward = transform
            .rotation
            .transform_vector(&Vector3::new(0.0, 0.0, -1.0));
        let right = transform
            .rotation
            .transform_vector(&Vector3::new(1.0, 0.0, 0.0));
        let up = transform
            .rotation
            .transform_vector(&Vector3::new(0.0, 1.0, 0.0));

        let mut direction = Vector3::new(0.0, 0.0, 0.0);
        if input_state.keys.forward {
            direction += forward;
        }
        if input_state.keys.back {
            direction -= forward;
        }
        if input_state.keys.right {
            direction += right;
        }
        if input_state.keys.left {
            direction -= right;
        }
        if input_state.keys.up {
            direction += up;
        }
        if input_state.keys.down {
            direction -= up;
        }

        if direction.magnitude() > 0.0 {
            velocity.translational = direction.normalize() * controller.speed;
        } else {
            //velocity.translational = Vector3::new(0.0, 0.0, 0.0);
        }

        velocity.translational.x -= velocity.translational.x * 0.1;
        velocity.translational.y -= velocity.translational.y * 0.1;
        velocity.translational.z -= velocity.translational.z * 0.1;
    }
}