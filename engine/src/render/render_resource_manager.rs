use bevy_ecs::prelude::*;

use crate::render::{
    material_resource::MaterialResource, mesh_resource::MeshResource,
    render_body_resource::RenderBodyResource, shader_resource::ShaderResource,
    texture_resource_manager::TextureResource,
};

#[derive(Resource)]
pub struct RenderResourceManager {
    pub mesh_manager: MeshResource,
    pub render_body_manager: RenderBodyResource,
    pub material_manager: MaterialResource,
    pub texture_manager: TextureResource,
    pub shader_manager: ShaderResource,
}

impl Default for RenderResourceManager {
    fn default() -> Self {
        Self::new()
    }
}

impl RenderResourceManager {
    pub fn new() -> Self {
        Self {
            mesh_manager: MeshResource::default(),
            render_body_manager: RenderBodyResource::default(),
            material_manager: MaterialResource::default(),
            texture_manager: TextureResource::default(),
            shader_manager: ShaderResource::default(),
        }
    }
}
