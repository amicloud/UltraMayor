use bevy_ecs::component::Component;
use nalgebra::{Quaternion, Vector3};

#[derive(Default, Component, Debug)]
pub struct TransformComponent {
    pub position: Vector3<f32>,
    pub rotation: Quaternion<f32>,
    pub scale: Vector3<f32>,
}
