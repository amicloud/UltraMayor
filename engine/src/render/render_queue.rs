use bevy_ecs::resource::Resource;

use crate::render::render_instance::RenderInstance;

#[derive(Resource, Default)]
pub struct RenderQueue {
    pub instances: Vec<RenderInstance>,
}
