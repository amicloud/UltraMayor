use crate::{handles::*, shader::Shader};

pub struct Material {
    pub shader: Shader,
    pub id: MaterialHandle,
    pub roughness: f32,
    pub albedo: Option<TextureHandle>,
    pub normal: Option<TextureHandle>,
    pub base_reflectance: f32,
}
