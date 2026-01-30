use std::collections::HashMap;
use std::ffi::OsStr;
use std::hash::{Hash, Hasher};

use glow::Context;

use crate::handles::ShaderHandle;
use crate::shader::Shader;

#[derive(Default)]
pub struct ShaderResource {
    shaders: HashMap<ShaderHandle, Shader>,
    shader_cache: HashMap<ShaderKey, ShaderHandle>,
}

#[derive(Hash, PartialEq, Eq)]
struct ShaderKey {
    vertex_path: String,
    fragment_path: String,
}

impl ShaderResource {
    pub fn get_or_load(
        &mut self,
        gl: &Context,
        vertex_src: &OsStr,
        fragment_src: &OsStr,
    ) -> ShaderHandle {
        let key = ShaderKey {
            vertex_path: vertex_src.to_string_lossy().into_owned(),
            fragment_path: fragment_src.to_string_lossy().into_owned(),
        };

        if let Some(handle) = self.shader_cache.get(&key) {
            return *handle;
        }

        let shader = Shader::new(gl, vertex_src, fragment_src);
        let handle = self.add_shader(shader, &key);
        handle
    }

    pub fn get_shader(&self, shader_id: ShaderHandle) -> Option<&Shader> {
        self.shaders.get(&shader_id)
    }

    fn add_shader(&mut self, shader: Shader, key: &ShaderKey) -> ShaderHandle {
        let id = self.make_hashed_id(key);
        self.shaders.insert(id, shader);
        self.shader_cache.insert(
            ShaderKey {
                vertex_path: key.vertex_path.clone(),
                fragment_path: key.fragment_path.clone(),
            },
            id,
        );
        id
    }

    fn make_hashed_id<T: Hash>(&self, value: &T) -> ShaderHandle {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        value.hash(&mut hasher);
        ShaderHandle(hasher.finish() as u32)
    }
}
