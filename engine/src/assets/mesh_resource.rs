use std::sync::{Arc, RwLock};

use slotmap::SlotMap;

use bevy_ecs::prelude::*;

use crate::{
    assets::{handles::MeshHandle, mesh::Mesh},
    render::renderer::Renderer,
};

#[derive(Default)]
pub struct MeshStorage {
    pub meshes: SlotMap<MeshHandle, Mesh>,
}

#[derive(Resource, Default, Clone)]
pub struct MeshResource(pub Arc<RwLock<MeshStorage>>);
impl MeshResource {
    pub fn read(&self) -> std::sync::RwLockReadGuard<'_, MeshStorage> {
        self.0.read().unwrap()
    }

    pub fn write(&self) -> std::sync::RwLockWriteGuard<'_, MeshStorage> {
        self.0.write().unwrap()
    }
}
impl MeshStorage {
    pub fn add_mesh(&mut self, mesh: Mesh) -> MeshHandle {
        self.meshes.insert(mesh)
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
