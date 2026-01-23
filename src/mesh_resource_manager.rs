use bevy_ecs::prelude::*;

#[derive(Default, Resource)]
pub struct MeshResourceManager {
    // HashMap of meshes
    meshes: std::collections::HashMap<u32, crate::mesh::Mesh>,

}

impl MeshResourceManager {
    pub fn new() -> Self {
        MeshResourceManager::default()
    }

    pub fn add_mesh(&mut self, mesh: crate::mesh::Mesh) -> u32{
        let id = mesh.id;
        self.meshes.insert(mesh.id, mesh);
        id
    }

    pub fn list_meshes(&self) -> Vec<u32> {
        self.meshes.keys().cloned().collect()
    }

    pub fn get_mesh(&self, mesh_id: u32) -> Option<&crate::mesh::Mesh> {
        self.meshes.get(&mesh_id)
    }
}