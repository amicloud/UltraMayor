use slotmap::SlotMap;

use bevy_ecs::prelude::*;

use crate::{
    assets::{handles::MeshHandle, mesh::Mesh},
    render::renderer::Renderer,
};

#[derive(Resource, Default)]
pub struct MeshResource {
    pub meshes: SlotMap<MeshHandle, Mesh>,
}

impl MeshResource {
    pub fn add_mesh(&mut self, mesh: Mesh) -> MeshHandle {
        let id = self.meshes.insert(mesh);
        id
    }

    #[allow(dead_code)]
    pub fn get_mesh(&self, mesh_id: MeshHandle) -> Option<&Mesh> {
        self.meshes.get(mesh_id)
    }

    pub fn get_mesh_mut(&mut self, mesh_id: MeshHandle) -> Option<&mut Mesh> {
        self.meshes.get_mut(mesh_id)
    }

    #[allow(dead_code)]
    pub fn remove_mesh(&mut self, mesh_id: MeshHandle, renderer: &mut Renderer) {
        if self.meshes.remove(mesh_id).is_some() {
            renderer.delete_mesh_gpu(mesh_id);
        }
    }
}
