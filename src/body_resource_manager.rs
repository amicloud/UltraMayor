use bevy_ecs::prelude::*;

use crate::body::Body;
#[derive(Default, Resource)]
pub(crate) struct BodyResourceManager {
    pub(crate) bodies: Vec<Body>,
}

impl BodyResourceManager {
    pub fn add_body(&mut self, body: Body) {
        self.bodies.push(body);
    }
}
