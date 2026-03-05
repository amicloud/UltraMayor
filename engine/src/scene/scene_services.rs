use std::sync::Arc;

use crate::{assets::{
    material_resource::MaterialResource, mesh_resource::MeshResource, shader_resource::ShaderResource, sound_resource::SoundResource, texture_resource::TextureResource
}, render::render_body_resource::RenderBodyResource};

pub struct SceneServices {
    pub meshes: MeshResource,
    pub textures: Arc<TextureResource>,
    pub shaders: Arc<ShaderResource>,
    pub sounds: Arc<SoundResource>,
    pub bodies: Arc<RenderBodyResource>,
    pub materials: Arc<MaterialResource>,
}

