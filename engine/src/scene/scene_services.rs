use std::sync::Arc;

use crate::assets::{
    mesh_resource::MeshResource, shader_resource::ShaderResource, sound_resource::SoundResource,
    texture_resource::TextureResource,
};

pub struct SceneServices {
    pub meshes: Arc<MeshResource>,
    pub textures: Arc<TextureResource>,
    pub shaders: Arc<ShaderResource>,
    pub sounds: Arc<SoundResource>,
}
