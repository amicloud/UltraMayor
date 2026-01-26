use bevy_ecs::prelude::*;

use crate::{handles::MaterialHandle, material::Material};

#[derive(Default, Resource)]
pub struct MaterialResourceManager {
    pub materials: std::collections::HashMap<MaterialHandle, Material>,
}

impl MaterialResourceManager {
    pub fn add_material(&mut self, material: Material) -> MaterialHandle {
        let id = material.id;
        self.materials.insert(id, material);
        id
    }

    pub fn get_material(&self, material_id: MaterialHandle) -> Option<&Material> {
        self.materials.get(&material_id)
    }

    #[allow(dead_code)]
    pub fn remove_material(&mut self, material_id: MaterialHandle) {
        self.materials.remove(&material_id);
    }
}
