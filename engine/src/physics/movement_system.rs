use bevy_ecs::prelude::*;
use glam::{Quat, Vec3};

use crate::{
    components::{
        physics_component::PhysicsComponent, transform_component::TransformComponent,
        velocity_component::VelocityComponent,
    },
    time_resource::TimeResource,
};
pub struct MovementSystem {}

impl MovementSystem {
    pub fn update(
        mut query: Query<(&mut TransformComponent, &VelocityComponent), Without<PhysicsComponent>>,
        time: Res<TimeResource>,
    ) {
        let delta_time = time.simulation_fixed_dt().as_secs_f32();
        for (mut transform, velocity) in query.iter_mut() {
            // Update position based on translational velocity
            transform.position =
                Self::apply_translation(&transform.position, &velocity.translational, delta_time);

            // Update rotation based on angular velocity
            let angular_velocity_magnitude = velocity.angular.length();
            if angular_velocity_magnitude > 0.0 {
                transform.rotation =
                    Self::apply_rotation(&transform.rotation, &velocity.angular, delta_time);
            }
        }
    }

    pub fn apply_rotation(rotation: &Quat, angular_velocity: &Vec3, delta_time: f32) -> Quat {
        let angular_velocity_magnitude = angular_velocity.length();
        if angular_velocity_magnitude == 0.0 {
            return *rotation;
        }

        let axis = angular_velocity / angular_velocity_magnitude;
        let angle = angular_velocity_magnitude * delta_time;
        let delta_rotation = Quat::from_axis_angle(axis, angle);
        delta_rotation * *rotation
    }

    fn apply_translation(position: &Vec3, translational_velocity: &Vec3, delta_time: f32) -> Vec3 {
        *position + translational_velocity * delta_time
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use assert_approx_eq::assert_approx_eq;

    use super::*;

    const DELTA_TIME: f32 = 1.0;

    #[test]
    fn apply_translation() {
        let position = Vec3::new(0.0, 0.0, 0.0);
        let velocity = Vec3::new(1.0, 2.0, 3.0);
        let new_position = MovementSystem::apply_translation(&position, &velocity, DELTA_TIME);
        assert_eq!(new_position, Vec3::new(1.0, 2.0, 3.0) * DELTA_TIME);
        let newer_position =
            MovementSystem::apply_translation(&new_position, &velocity, DELTA_TIME);
        assert_eq!(newer_position, Vec3::new(2.0, 4.0, 6.0) * DELTA_TIME);
    }

    #[test]
    fn apply_rotation_x() {
        let rotation = Quat::IDENTITY;
        let angular_velocity = Vec3::new(PI, 0.0, 0.0);
        let new_rotation = MovementSystem::apply_rotation(&rotation, &angular_velocity, DELTA_TIME);
        assert_approx_eq!(new_rotation.x, 1.0 * DELTA_TIME, 1e-6);
    }

    #[test]
    fn apply_rotation_y() {
        let rotation = Quat::IDENTITY;
        let angular_velocity = Vec3::new(0.0, PI, 0.0);
        let new_rotation = MovementSystem::apply_rotation(&rotation, &angular_velocity, DELTA_TIME);
        assert_approx_eq!(new_rotation.y, 1.0 * DELTA_TIME, 1e-6);
    }

    #[test]
    fn apply_rotation_z() {
        let rotation = Quat::IDENTITY;
        let angular_velocity = Vec3::new(0.0, 0.0, PI);
        let new_rotation = MovementSystem::apply_rotation(&rotation, &angular_velocity, DELTA_TIME);
        assert_approx_eq!(new_rotation.z, 1.0 * DELTA_TIME, 1e-6);
    }
}
