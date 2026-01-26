use bevy_ecs::prelude::*;

use crate::{material_resource_manager, mesh_resource_manager, texture_resource_manager};

#[derive(Default, Resource)]
pub struct RenderDataManager {
    pub mesh_manager: mesh_resource_manager::MeshResourceManager,
    pub material_manager: material_resource_manager::MaterialResourceManager,
    pub texture_manager: texture_resource_manager::TextureResourceManager,
}
