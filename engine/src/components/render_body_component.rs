use bevy_ecs::prelude::*;

use crate::{TransformComponent, handles::RenderBodyHandle};

#[derive(Component, Debug, Clone, Copy)]
#[require(TransformComponent)]
/// Component that links an entity to its render body, which contains the necessary information for rendering.
pub struct RenderBodyComponent {
    pub render_body_id: RenderBodyHandle,
}
