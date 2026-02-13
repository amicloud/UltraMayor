use bevy_ecs::prelude::*;
use glam::Vec3;
use std::collections::HashMap;

use crate::{
    dynamic_aabb_tree::{DynamicAabbTree, NodeId},
    mesh::AABB,
};

#[derive(Debug, Clone, Copy)]
pub struct Contact {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub normal: Vec3,        // Direction from A to B
    pub penetration: f32,    // Depth of overlap
    pub contact_point: Vec3, // Point of contact in world space
}

#[derive(Debug, Clone)]
pub struct ContactManifold {
    pub contacts: Vec<Contact>,
    pub normal: Vec3,
}

#[derive(Resource, Default)]
pub struct PhysicsResource {
    pub world_aabbs: HashMap<Entity, AABB>,
    pub broadphase: DynamicAabbTree,
    pub entity_node: HashMap<Entity, NodeId>,
}

#[derive(Resource, Default)]
pub struct CollisionFrameData {
    pub delta_time: f32,
    pub candidate_pairs: Vec<(Entity, Entity)>,
    pub contacts: Vec<Contact>,
    pub manifolds: HashMap<(Entity, Entity), ContactManifold>,
}

impl CollisionFrameData {
    pub fn clear(&mut self) {
        self.delta_time = 0.0;
        self.candidate_pairs.clear();
        self.contacts.clear();
        self.manifolds.clear();
    }
}
