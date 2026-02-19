use glam::Mat4;

use crate::assets::handles::{MaterialHandle, MeshHandle};

#[derive(Clone)]
pub struct RenderBodyPart {
    pub mesh_id: MeshHandle,
    pub material_id: MaterialHandle,
    pub local_transform: Mat4,
}

#[derive(Clone)]
pub struct RenderBody {
    pub parts: Vec<RenderBodyPart>,
}

impl RenderBody {
    pub fn new(parts: Vec<RenderBodyPart>) -> Self {
        Self { parts }
    }
}
