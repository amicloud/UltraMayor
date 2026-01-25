use bevy_ecs::prelude::*;

use crate::{mesh::Mesh, renderer::Renderer};

#[derive(Default, Resource)]
pub struct MeshResourceManager {
    pub meshes: std::collections::HashMap<u32, Mesh>,
}

impl MeshResourceManager {
    pub fn add_mesh(&mut self, mut mesh: Mesh, gl: &glow::Context) -> u32 {
        mesh.upload_to_gpu(gl);
        let id = mesh.id;
        self.meshes.insert(id, mesh);
        id
    }

    #[allow(dead_code)]
    pub fn get_mesh(&self, mesh_id: u32) -> Option<&Mesh> {
        self.meshes.get(&mesh_id)
    }

    pub fn get_mesh_mut(&mut self, mesh_id: u32) -> Option<&mut Mesh> {
        self.meshes.get_mut(&mesh_id)
    }

    #[allow(dead_code)]
    pub fn remove_mesh(&mut self, mesh_id: u32, renderer: &Renderer) {
        if let Some(mut mesh) = self.meshes.remove(&mesh_id) {
            renderer.delete_mesh_gpu(&mut mesh);
        }
    }
}
