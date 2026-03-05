use bevy_ecs::prelude::*;

use crate::scene::scene::Scene;

#[derive(Resource, Default)]
pub struct SceneChangerResource {
    pending_scene: Option<Scene>,
}

impl SceneChangerResource {
    pub fn request_change(&mut self, scene: Scene) {
        if self.pending_scene.is_some() {
            log::warn!("Scene change already pending; replacing with newer request");
        }
        self.pending_scene = Some(scene);
    }

    pub fn take_pending(&mut self) -> Option<Scene> {
        self.pending_scene.take()
    }

    pub fn has_pending(&self) -> bool {
        self.pending_scene.is_some()
    }
}

