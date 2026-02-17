use glam::Mat4;

use crate::handles::{MaterialHandle, MeshHandle, RenderBodyHandle};

#[derive(Clone)]
pub struct RenderBodyPart {
    pub mesh_id: MeshHandle,
    pub material_id: MaterialHandle,
    pub local_transform: Mat4,
}

#[derive(Clone)]
pub struct RenderBody {
    pub id: RenderBodyHandle,
    pub parts: Vec<RenderBodyPart>,
}

impl RenderBody {
    pub fn new(id: RenderBodyHandle, parts: Vec<RenderBodyPart>) -> Self {
        Self { id, parts }
    }
}
