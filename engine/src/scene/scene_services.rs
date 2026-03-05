use bevy_ecs::prelude::*;

use crate::{
    assets::{
        material_resource::MaterialResource, mesh_resource::MeshResource,
        shader_resource::ShaderResource, sound_resource::SoundResource,
        texture_resource::TextureResource,
    },
    render::render_body_resource::RenderBodyResource,
};

#[derive(Resource, Clone)]
pub struct SceneServices {
    pub meshes: MeshResource,
    pub textures: TextureResource,
    pub shaders: ShaderResource,
    pub sounds: SoundResource,
    pub bodies: RenderBodyResource,
    pub materials: MaterialResource,
}
