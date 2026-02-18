use bevy_ecs::prelude::*;

#[derive(EntityEvent)]
pub struct CollisionEvent {
    pub entity: Entity,
    pub event_type: CollisionEventType,
    pub collision_info: CollisionInfo,
    pub other_entity: Entity,
}

#[derive(Debug, Clone)]
pub struct CollisionInfo {
    pub normal: glam::Vec3,        // Direction from A to B
    pub penetration: f32,          // Depth of overlap
    pub contact_point: glam::Vec3, // Point of contact in world space
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionEventType {
    Enter,
    Stay,
    Exit,
}