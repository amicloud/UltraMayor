use crate::{
    body::Body,
    body_resource_manager::BodyResourceManager,
    mesh_resource_manager::MeshResourceManager,
    transform_component::TransformComponent,
};
use crate::mesh_component::MeshComponent;
use bevy_ecs::prelude::*;

#[derive(Default)]
pub struct BodyBuilderSystem {}

impl BodyBuilderSystem {
    pub fn build_bodies(
        query: Query<(&TransformComponent, &MeshComponent)>,
        mut body_manager: ResMut<BodyResourceManager>,
        mesh_manager: Res<MeshResourceManager>,
    ) {
        body_manager.bodies.clear();
        for (transform, mesh) in query.iter() {
            // Logic to build body from transform and mesh
            println!(
                "Building body with Transform: {:?} and Mesh ID: {}",
                transform, mesh.mesh_id
            );
            let mesh = mesh_manager.get_mesh(mesh.mesh_id).unwrap();
            let mut new_body = Body::default();
            new_body.mesh = mesh.clone();
            new_body.position = transform.position;
            new_body.rotation = transform.rotation;
            new_body.scale = transform.scale;
            body_manager.add_body(new_body);
        }
    }
}
