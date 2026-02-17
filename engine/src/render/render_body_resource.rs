use bevy_ecs::prelude::*;

use crate::{handles::RenderBodyHandle, render::render_body::RenderBody};

#[derive(Resource, Default)]
pub struct RenderBodyResource {
    pub render_bodies: std::collections::HashMap<RenderBodyHandle, RenderBody>,
}

impl RenderBodyResource {
    pub fn add_render_body(&mut self, render_body: RenderBody) -> RenderBodyHandle {
        let id = render_body.id;
        self.render_bodies.insert(id, render_body);
        id
    }

    pub fn get_render_body(&self, render_body_id: RenderBodyHandle) -> Option<&RenderBody> {
        self.render_bodies.get(&render_body_id)
    }

    #[allow(dead_code)]
    pub fn get_render_body_mut(
        &mut self,
        render_body_id: RenderBodyHandle,
    ) -> Option<&mut RenderBody> {
        self.render_bodies.get_mut(&render_body_id)
    }

    #[allow(dead_code)]
    pub fn remove_render_body(&mut self, render_body_id: RenderBodyHandle) {
        self.render_bodies.remove(&render_body_id);
    }
}
