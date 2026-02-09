use bevy_ecs::component::Component;

#[derive(Clone, Copy)]
pub enum PhysicsType {
    Static,
    Dynamic,
    Kinematic,
}

#[derive(Component, Clone, Copy)]
pub struct PhysicsComponent {
    pub physics_type: PhysicsType,
    pub mass: f32,
    pub friction: f32,
    pub drag_coefficient: f32,
    pub angular_drag_coefficient: f32,
    pub restitution: f32,
    pub local_inertia: glam::Mat3,
}
