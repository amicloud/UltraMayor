use bevy_ecs::resource::Resource;
use glam::{Quat, Vec3};

use crate::WorldBasis;

#[derive(Resource, Debug)]
pub struct Gravity {
    pub gravity_normal: Vec3,
    pub gravity_magnitude: f32,
}

impl Default for Gravity {
    fn default() -> Self {
        Self {
            gravity_normal: -WorldBasis::canonical().up(),
            gravity_magnitude: 9.81,
        }
    }
}

impl Gravity {
    #[allow(dead_code)]
    pub fn new(gravity_direction: Vec3, gravity_magnitude: f32) -> Self {
        Self {
            gravity_normal: gravity_direction.normalize(),
            gravity_magnitude,
        }
    }

    #[allow(dead_code)]
    pub fn rotate_gravity(&mut self, rotation: Quat) {
        self.gravity_normal = (rotation * self.gravity_normal).normalize();
    }

    #[allow(dead_code)]
    /// This generally should not be used directly, as it can lead to gimbal lock. 
    /// Use `rotate_gravity` with a custom rotation or `rotate_gravity_around_axis` instead.
    pub fn rotate_gravity_euler(&mut self, euler_angles: Vec3) {
        let rotation = Quat::from_euler(
            glam::EulerRot::XYZ,
            euler_angles.x,
            euler_angles.y,
            euler_angles.z,
        );
        self.rotate_gravity(rotation);
    }

    #[allow(dead_code)]
    pub fn rotate_gravity_around_axis(&mut self, axis: Vec3, angle: f32) {
        let rotation = Quat::from_axis_angle(axis.normalize(), angle);
        self.rotate_gravity(rotation);
    }

    #[allow(dead_code)]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn gravity_vector(&self) -> Vec3 {
        self.gravity_normal * self.gravity_magnitude
    }

    pub fn up(&self) -> Vec3 {
        -self.gravity_normal
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    #[test]
    fn gravity_rotate_gravity() {
        let mut gravity = Gravity::default();
        // Rotate 90 degrees around Y axis, which should turn gravity from pointing down to pointing right.
        let rotation = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        gravity.rotate_gravity(rotation);
        let right = WorldBasis::canonical().right();
        assert_approx_eq!(gravity.gravity_normal.x, right.x, 1e-5);
        assert_approx_eq!(gravity.gravity_normal.y, right.y, 1e-5);
        assert_approx_eq!(gravity.gravity_normal.z, right.z, 1e-5);
    }

    #[test]
    fn gravity_rotate_gravity_euler() {
        let mut gravity = Gravity::default();
        // Rotate 90 degrees around Y axis, which should turn gravity from pointing down to pointing right.
        gravity.rotate_gravity_euler(Vec3::new(0.0, std::f32::consts::FRAC_PI_2, 0.0));
        let right = WorldBasis::canonical().right();
        assert_approx_eq!(gravity.gravity_normal.x, right.x, 1e-5);
        assert_approx_eq!(gravity.gravity_normal.y, right.y, 1e-5);
        assert_approx_eq!(gravity.gravity_normal.z, right.z, 1e-5);
    }

    #[test]
    fn gravity_rotate_gravity_around_axis() {
        let mut gravity = Gravity::default();
        // Rotate 90 degrees around Y axis, which should turn gravity from pointing down to pointing right.
        gravity.rotate_gravity_around_axis(Vec3::Y, std::f32::consts::FRAC_PI_2);
        let right = WorldBasis::canonical().right();
        assert_approx_eq!(gravity.gravity_normal.x, right.x, 1e-5);
        assert_approx_eq!(gravity.gravity_normal.y, right.y, 1e-5);
        assert_approx_eq!(gravity.gravity_normal.z, right.z, 1e-5);
    }
}
