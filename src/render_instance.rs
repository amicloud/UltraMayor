use crate::handles::*;
use nalgebra::Matrix4;

#[derive(Clone, Debug)]
pub struct RenderInstance {
    pub mesh_id: MeshHandle,
    pub transform: Matrix4<f32>,
    pub material_id: MaterialHandle,
}
