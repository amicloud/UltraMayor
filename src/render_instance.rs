use nalgebra::Matrix4;

#[derive(Clone, Debug)]
pub struct RenderInstance {
    pub mesh_id: u32,
    pub transform: Matrix4<f32>,
}
