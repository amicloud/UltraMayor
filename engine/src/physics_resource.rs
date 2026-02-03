use bevy_ecs::prelude::*;
use glam::Vec3;
use std::collections::HashMap;

use crate::mesh::AABB;

pub struct Impulse {
    pub entity: Entity,
    pub linear: Vec3,
    pub angular: Vec3,
}

#[derive(Debug, Clone)]
pub struct Contact {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub normal: Vec3,      // Direction from A to B
    pub penetration: f32,  // Depth of overlap
}

#[derive(Default, Resource)]
pub struct PhysicsResource {
    pub impulses: Vec<Impulse>,
    pub world_aabbs: HashMap<Entity, AABB>,
    pub contacts: Vec<Contact>,
}

impl PhysicsResource {
    pub fn add_impulse(&mut self, entity: Entity, linear: Vec3, angular: Vec3) {
        self.impulses.push(Impulse {
            entity,
            linear,
            angular,
        });
    }

    pub fn add_contact(&mut self, contact: Contact) {
        self.contacts.push(contact);
    }
}
