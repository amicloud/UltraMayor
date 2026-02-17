use bevy_ecs::prelude::*;

use crate::handles::RenderBodyHandle;

#[derive(Component, Debug, Clone, Copy)]
pub struct RenderBodyComponent {
    pub render_body_id: RenderBodyHandle,
}
