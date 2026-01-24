use bevy_ecs::component::Component;
use nalgebra::{Quaternion, Vector3};

#[derive(Component, Debug)]
pub struct TransformComponent {
    pub position: Vector3<f32>,
    pub rotation: Quaternion<f32>,
    pub scale: Vector3<f32>,
}

impl Default for TransformComponent {
    fn default() -> Self {
        Self {
            position: Vector3::new(0.0, 0.0, 0.0),
            rotation: Quaternion::identity(),
            scale: Vector3::new(1.0, 1.0, 1.0),
        }
    }
}
