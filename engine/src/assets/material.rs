use crate::assets::{handles::*, shader::UniformValue};

pub struct MaterialDesc {
    pub shader: ShaderHandle,
    pub params: MaterialParams,
}

type MaterialParams = Vec<(String, UniformValue)>;

impl MaterialDesc {
    pub fn new(shader: ShaderHandle, params: MaterialParams) -> Self {
        Self { shader, params }
    }
}

pub struct Material {
    pub desc: MaterialDesc,
}

impl Material {
    pub fn new(desc: MaterialDesc) -> Self {
        Self { desc }
    }
}
