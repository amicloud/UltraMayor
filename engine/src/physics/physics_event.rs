use bevy_ecs::prelude::*;

use crate::physics::physics_resource::Contact;

#[derive(EntityEvent)]
pub struct PhysicsEvent {
    pub entity: Entity,
    pub event_type: PhysicsEventType,
    pub collision_info: PhysicsEventInfo,
    pub other_entity: Entity,
}

#[derive(Debug, Clone)]
pub struct PhysicsEventInfo {
    pub normal: glam::Vec3,
    pub contacts: Vec<Contact>,
    pub relative_normal_speed: f32,
    pub impact_impulse: f32,
    pub impact_energy: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhysicsEventType {
    Hit,
    Stay,
}
