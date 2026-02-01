use bevy_ecs::component::Component;

use crate::mesh::AABB;

#[derive(Clone, Copy)]
enum CollisionLayer {
    Default,
    Player,
    Enemy,
    Environment,
}

#[derive(Clone, Copy, Component)]
pub struct ColliderComponent {
    aabb: AABB,
    layer: CollisionLayer,
}

impl ColliderComponent {
    pub fn new(aabb: AABB, layer: CollisionLayer) -> Self {
        Self { aabb, layer }
    }

    pub fn intersects(&self, other: &ColliderComponent) -> bool {
        (self.aabb.min.x <= other.aabb.max.x && self.aabb.max.x >= other.aabb.min.x)
            && (self.aabb.min.y <= other.aabb.max.y && self.aabb.max.y >= other.aabb.min.y)
            && (self.aabb.min.z <= other.aabb.max.z && self.aabb.max.z >= other.aabb.min.z)
    }
}
