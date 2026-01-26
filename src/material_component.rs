use bevy_ecs::prelude::*;

use crate::handles::MaterialHandle;

#[derive(Component, Debug, Copy, Clone)]
pub struct MaterialComponent {
    pub material_id: MaterialHandle,
}
