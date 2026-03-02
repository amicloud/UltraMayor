use bevy_ecs::prelude::*;
use glam::Vec3;
use std::collections::HashMap;

use crate::{
    assets::mesh::Aabb,
    physics::{self, collision_system::OrderedEntityPair},
};
use physics::{
    dynamic_aabb_tree::{DynamicAabbTree, NodeId},
    physics_system::ContactConstraint,
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
    pub relative_normal_speed: f32,
    pub impact_impulse: f32,
    pub impact_energy: f32,
}

#[derive(Resource, Default)]
pub struct PhysicsResource {
    pub world_aabbs: HashMap<Entity, Aabb>,
    pub broadphase: DynamicAabbTree,
    pub entity_node: HashMap<Entity, NodeId>,
}

#[derive(Resource, Default)]
pub struct PhysicsFrameData {
    pub constraints: Vec<ContactConstraint>,
    pub corrections: HashMap<Entity, Vec3>,
}

#[derive(Resource, Default)]
pub struct CollisionFrameData {
    pub delta_time: f32,
    pub candidate_pairs: Vec<(Entity, Entity)>,
    pub manifolds: ManifoldVec,
    pub previous_manifolds: ManifoldVec,
}

pub struct ManifoldEntry {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub manifold: ContactManifold,
}

#[derive(Default)]
pub struct ManifoldVec(Vec<ManifoldEntry>);
impl ManifoldVec {
    pub fn get(&self, pair: OrderedEntityPair) -> Option<&ContactManifold> {
        self.0
            .iter()
            .find(|entry| entry.entity_a == pair.0 && entry.entity_b == pair.1)
            .map(|entry| &entry.manifold)
    }

    pub fn get_mut(&mut self, pair: OrderedEntityPair) -> Option<&mut ContactManifold> {
        self.0
            .iter_mut()
            .find(|entry| entry.entity_a == pair.0 && entry.entity_b == pair.1)
            .map(|entry| &mut entry.manifold)
    }

    fn clear(&mut self) {
        self.0.clear();
    }

    pub fn iter(&self) -> impl Iterator<Item = &ManifoldEntry> {
        self.0.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut ManifoldEntry> {
        self.0.iter_mut()
    }

    pub fn push(&mut self, pair: OrderedEntityPair, manifold: ContactManifold) {
        self.0.push(ManifoldEntry {
            entity_a: pair.0,
            entity_b: pair.1,
            manifold,
        });
    }
}

impl CollisionFrameData {
    pub fn clear(&mut self) {
        self.delta_time = 0.0;
        self.candidate_pairs.clear();
        self.previous_manifolds = std::mem::take(&mut self.manifolds);
        self.manifolds.clear();
    }
}

impl PhysicsFrameData {
    pub fn clear(&mut self) {
        self.constraints.clear();
    }
}
