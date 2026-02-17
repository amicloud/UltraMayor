use std::{
    collections::{HashMap, hash_map::DefaultHasher},
    hash::{Hash, Hasher},
};

use crate::{handles::*, render::shader::UniformValue};

pub struct MaterialDesc {
    pub shader: ShaderHandle,
    pub params: HashMap<String, UniformValue>,
}

impl Hash for MaterialDesc {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.shader.hash(state);
        for (key, value) in &self.params {
            key.hash(state);
            match value {
                UniformValue::Float(f) => {
                    f.to_bits().hash(state);
                }
                UniformValue::Vec3(v) => {
                    v.x.to_bits().hash(state);
                    v.y.to_bits().hash(state);
                    v.z.to_bits().hash(state);
                }
                UniformValue::Mat4(m) => {
                    for i in 0..4 {
                        for j in 0..4 {
                            m.col(i)[j].to_bits().hash(state);
                        }
                    }
                }
                UniformValue::Int(i) => {
                    i.hash(state);
                }
                UniformValue::Texture { handle, unit } => {
                    handle.hash(state);
                    unit.hash(state);
                }
            }
        }
    }
}

impl MaterialDesc {
    pub fn new(shader: ShaderHandle, params: HashMap<String, UniformValue>) -> Self {
        Self { shader, params }
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
