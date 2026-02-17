use bevy_ecs::prelude::{Query, Res, ResMut};

use crate::{
    components::{
        render_body_component::RenderBodyComponent, transform_component::TransformComponent,
    },
    render::{
        render_instance::RenderInstance, render_queue::RenderQueue,
        render_resource_manager::RenderResourceManager,
    },
};

pub struct RenderSystem {}

impl RenderSystem {
    pub fn build_render_queue(
        query: Query<(&TransformComponent, &RenderBodyComponent)>,
        render_resources: Res<RenderResourceManager>,
        mut queue: ResMut<RenderQueue>,
    ) {
        queue.instances.clear();

        for (transform, render_body) in &query {
            let body = render_resources
                .render_body_manager
                .get_render_body(render_body.render_body_id)
                .expect("RenderBody not found");

            let world_transform = transform.to_mat4();
            for part in &body.parts {
                queue.instances.push(RenderInstance {
                    mesh_id: part.mesh_id,
                    transform: world_transform * part.local_transform,
                    material_id: part.material_id,
                });
            }
        }
    }
}
