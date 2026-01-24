use bevy_ecs::component::Component;
use nalgebra::Vector3;

#[derive(Default, Component)]
pub struct VelocityComponent {
    pub translational: Vector3<f32>,
    pub angular: Vector3<f32>,
}
