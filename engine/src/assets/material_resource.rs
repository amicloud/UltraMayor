use bevy_ecs::prelude::*;
use slotmap::SlotMap;

use crate::assets::{handles::MaterialHandle, material::Material};

#[derive(Resource, Default)]
pub struct MaterialResource {
    pub materials: SlotMap<MaterialHandle, Material>,
}

impl MaterialResource {
    pub fn add_material(&mut self, material: Material) -> MaterialHandle {
        let id = self.materials.insert(material);
        id
    }

    pub fn get_material(&self, material_id: MaterialHandle) -> Option<&Material> {
        self.materials.get(material_id)
    }

    #[allow(dead_code)]
    pub fn remove_material(&mut self, material_id: MaterialHandle) {
        self.materials.remove(material_id);
    }
}
