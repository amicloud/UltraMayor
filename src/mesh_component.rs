use bevy_ecs::prelude::*;

use crate::handles::MeshHandle;

#[derive(Component, Debug, Copy, Clone)]
pub struct MeshComponent {
    pub mesh_id: MeshHandle,
}
