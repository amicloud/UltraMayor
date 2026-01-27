use bevy_ecs::prelude::*;

use crate::{
    material_resource_manager::MaterialResourceManager, mesh_resource_manager::MeshResourceManager,
    texture_resource_manager::TextureResourceManager,
};

#[derive(Resource)]
pub struct RenderDataManager {
    pub mesh_manager: MeshResourceManager,
    pub material_manager: MaterialResourceManager,
    pub texture_manager: TextureResourceManager,
}

impl RenderDataManager {
    pub fn new() -> Self {
        Self {
            mesh_manager: MeshResourceManager::default(),
            material_manager: MaterialResourceManager::default(),
            texture_manager: TextureResourceManager::default(),
        }
    }
}
