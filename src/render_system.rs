use crate::{
    material_component::MaterialComponent, mesh_component::MeshComponent,
    render_instance::RenderInstance, render_queue::RenderQueue,
    transform_component::TransformComponent,
};

use bevy_ecs::prelude::{Query, ResMut};
pub struct RenderSystem {}

impl RenderSystem {
    pub fn extract_render_data(
        query: Query<(&TransformComponent, &MeshComponent, &MaterialComponent)>,
        mut queue: ResMut<RenderQueue>,
    ) {
        queue.instances.clear();

        for (transform, mesh, material) in &query {
            queue.instances.push(RenderInstance {
                mesh_id: mesh.mesh_id,
                transform: transform.to_mat4(),
                material_id: material.material_id,
            });
        }
    }
}
