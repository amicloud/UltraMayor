use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
};

use crate::{handles::*, shader::Shader};

#[derive(Hash)]
pub struct MaterialDesc {
    pub shader: Shader,
    pub roughness: u32,
    pub albedo: TextureHandle,
    pub normal: Option<TextureHandle>,
    pub base_reflectance: u32,
}

impl MaterialDesc {
    pub fn new(
        shader: Shader,
        roughness: f32,
        base_reflectance: f32,
        albedo: TextureHandle,
        normal: Option<TextureHandle>,
    ) -> Self {
        Self {
            shader,
            roughness: roughness.to_bits(),
            base_reflectance: base_reflectance.to_bits(),
            albedo,
            normal,
        }
    }
}

pub struct Material {
    pub id: MaterialHandle,
    pub desc: MaterialDesc,
}

impl Material {
    pub fn new(desc: MaterialDesc) -> Self {
        // Make a hash of all the arguments
        let id: MaterialHandle = {
            let mut hasher = DefaultHasher::new();
            desc.hash(&mut hasher);
            MaterialHandle(hasher.finish() as u32)
        };

        Self { id, desc }
    }
}
