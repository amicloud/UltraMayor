use bevy_ecs::prelude::*;

use crate::mesh::Mesh;

#[derive(Default, Resource)]
pub struct MeshResourceManager {
    // HashMap of meshes
    meshes: std::collections::HashMap<u32, Mesh>,
}

impl MeshResourceManager {
    pub fn add_mesh(&mut self, mesh: Mesh) -> u32 {
        let id = mesh.id;
        self.meshes.insert(mesh.id, mesh);
        id
    }

    pub fn get_mesh(&self, mesh_id: u32) -> Option<&Mesh> {
        self.meshes.get(&mesh_id)
    }
}
