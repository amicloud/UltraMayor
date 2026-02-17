use std::collections::HashMap;

use bevy_ecs::prelude::*;

use crate::{handles::MeshHandle, mesh::Mesh, render::renderer::Renderer};

#[derive(Resource, Default)]
pub struct MeshResource {
    pub meshes: HashMap<MeshHandle, Mesh>,
}

impl MeshResource {
    pub fn add_mesh(&mut self, mesh: Mesh) -> MeshHandle {
        let id = mesh.id;
        self.meshes.insert(id, mesh);
        id
    }

    #[allow(dead_code)]
    pub fn get_mesh(&self, mesh_id: MeshHandle) -> Option<&Mesh> {
        self.meshes.get(&mesh_id)
    }

    pub fn get_mesh_mut(&mut self, mesh_id: MeshHandle) -> Option<&mut Mesh> {
        self.meshes.get_mut(&mesh_id)
    }

    #[allow(dead_code)]
    pub fn remove_mesh(&mut self, mesh_id: MeshHandle, renderer: &mut Renderer) {
        if self.meshes.remove(&mesh_id).is_some() {
            renderer.delete_mesh_gpu(mesh_id);
        }
    }
}
