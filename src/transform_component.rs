use bevy_ecs::component::Component;
use nalgebra::{Quaternion, Vector3};

#[derive(Default, Component)]
pub struct TransformComponent {
    pub position: Vector3<f64>,
    pub rotation: Quaternion<f64>,
    pub scale: Vector3<f64>,
}
