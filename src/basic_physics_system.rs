use crate::transform_component::TransformComponent;
use crate::velocity_component::VelocityComponent;
use bevy_ecs::prelude::*;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
pub struct BasicPhysicsSystem {}

impl BasicPhysicsSystem {
    pub fn update(mut query: Query<(&mut TransformComponent, &VelocityComponent)>) {
        let delta_time = 1.0 / 60.0; // Assuming a fixed time step of 1/60 seconds
        for (mut transform, velocity) in query.iter_mut() {
            // Update position based on translational velocity
            transform.position =
                Self::apply_translation(&transform.position, &velocity.translational, delta_time);

            // Update rotation based on angular velocity
            let angular_velocity_magnitude = velocity.angular.norm();
            if angular_velocity_magnitude > 0.0 {
                transform.rotation =
                    Self::apply_rotation(&transform.rotation, &velocity.angular, delta_time);
            }
            println![
                "Transform: position={:?}, rotation={:?}",
                transform.position, transform.rotation
            ];
        }
    }

    fn apply_rotation(
        rotation: &Quaternion<f64>,
        angular_velocity: &Vector3<f64>,
        delta_time: f64,
    ) -> Quaternion<f64> {
        let angular_velocity_magnitude = angular_velocity.norm();
        if angular_velocity_magnitude == 0.0 {
            return *rotation;
        }

        let axis = angular_velocity / angular_velocity_magnitude;
        let angle = angular_velocity_magnitude * delta_time;
        let delta_rotation =
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle);
        let current_rotation = UnitQuaternion::from_quaternion(*rotation);
        let new_rotation = delta_rotation * current_rotation;
        new_rotation.into_inner()
    }

    fn apply_translation(
        position: &Vector3<f64>,
        translational_velocity: &Vector3<f64>,
        delta_time: f64,
    ) -> Vector3<f64> {
        *position + translational_velocity * delta_time
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use assert_approx_eq::assert_approx_eq;
    use std::f64::consts::PI;

    const DELTA_TIME: f64 = 1.0;

    #[test]
    fn test_apply_translation() {
        let position = Vector3::new(0.0, 0.0, 0.0);
        let velocity = Vector3::new(1.0, 2.0, 3.0);
        let new_position = BasicPhysicsSystem::apply_translation(&position, &velocity, DELTA_TIME);
        assert_eq!(new_position, Vector3::new(1.0, 2.0, 3.0) * DELTA_TIME);
        let newer_position =
            BasicPhysicsSystem::apply_translation(&new_position, &velocity, DELTA_TIME);
        assert_eq!(newer_position, Vector3::new(2.0, 4.0, 6.0) * DELTA_TIME);
    }

    #[test]
    fn test_apply_rotation_x() {
        let rotation = Quaternion::identity();
        let angular_velocity = Vector3::new(PI, 0.0, 0.0);
        let new_rotation =
            BasicPhysicsSystem::apply_rotation(&rotation, &angular_velocity, DELTA_TIME);
        assert_approx_eq!(new_rotation[0], 1.0 * DELTA_TIME, 1e-6);
    }

    #[test]
    fn test_apply_rotation_y() {
        let rotation = Quaternion::identity();
        let angular_velocity = Vector3::new(0.0, PI, 0.0);
        let new_rotation =
            BasicPhysicsSystem::apply_rotation(&rotation, &angular_velocity, DELTA_TIME);
        assert_approx_eq!(new_rotation[1], 1.0 * DELTA_TIME, 1e-6);
    }

    #[test]
    fn test_apply_rotation_z() {
        let rotation = Quaternion::identity();
        let angular_velocity = Vector3::new(0.0, 0.0, PI);
        let new_rotation =
            BasicPhysicsSystem::apply_rotation(&rotation, &angular_velocity, DELTA_TIME);
        assert_approx_eq!(new_rotation[2], 1.0 * DELTA_TIME, 1e-6);
    }
}
