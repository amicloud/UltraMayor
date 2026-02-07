use bevy_ecs::prelude::*;
use glam::Vec3;
use std::collections::HashMap;

use crate::mesh::AABB;

#[derive(Debug, Clone, Copy)]
pub struct Contact {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub normal: Vec3,     // Direction from A to B
    pub penetration: f32, // Depth of overlap
    pub contact_point: Vec3, // Point of contact in world space
}

#[derive(Debug, Clone)]
pub struct ContactManifold {
    pub contacts: Vec<Contact>,
}

#[derive(Default, Resource)]
pub struct PhysicsResource {
    pub world_aabbs: HashMap<Entity, AABB>,
    pub contacts: Vec<Contact>,
    pub manifolds: HashMap<(Entity, Entity), ContactManifold>,
}

impl PhysicsResource {
    pub fn add_contact(&mut self, contact: Contact) {
        self.contacts.push(contact);
    }

    pub fn are_in_contact(&self, entity_a: Entity, entity_b: Entity) -> bool {
        self.contacts.iter().any(|contact| {
            (contact.entity_a == entity_a && contact.entity_b == entity_b)
                || (contact.entity_a == entity_b && contact.entity_b == entity_a)
        })
    }
}
