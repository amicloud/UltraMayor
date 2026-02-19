use bevy_ecs::prelude::*;
use slotmap::SlotMap;

use crate::{RenderBodyHandle, render::render_body::RenderBody};

#[derive(Resource, Default)]
pub struct RenderBodyResource {
    pub render_bodies: SlotMap<RenderBodyHandle, RenderBody>,
}

impl RenderBodyResource {
    pub fn add_render_body(&mut self, render_body: RenderBody) -> RenderBodyHandle {
        self.render_bodies.insert(render_body)
    }

    pub fn get_render_body(&self, render_body_id: RenderBodyHandle) -> Option<&RenderBody> {
        self.render_bodies.get(render_body_id)
    }

    #[allow(dead_code)]
    pub fn get_render_body_mut(
        &mut self,
        render_body_id: RenderBodyHandle,
    ) -> Option<&mut RenderBody> {
        self.render_bodies.get_mut(render_body_id)
    }

    #[allow(dead_code)]
    pub fn remove_render_body(&mut self, render_body_id: RenderBodyHandle) {
        self.render_bodies.remove(render_body_id);
    }
}
