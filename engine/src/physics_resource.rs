use bevy_ecs::prelude::*;
use glam::Vec3;
use std::collections::HashMap;

use crate::mesh::AABB;

pub struct Impulse {
    pub entity: Entity,
    pub linear: Vec3,
    pub angular: Vec3,
}

#[derive(Default, Resource)]
pub struct PhysicsResource {
    pub impulses: Vec<Impulse>,
    pub world_aabbs: HashMap<Entity, AABB>,
}

impl PhysicsResource {
    pub fn add_impulse(&mut self, entity: Entity, linear: Vec3, angular: Vec3) {
        self.impulses.push(Impulse {
            entity,
            linear,
            angular,
        });
    }
}
