mod settings;
mod camera_controller;
mod player_controller;
mod input_controller;

use nalgebra::Vector3;
use rand::random_range;
use std::ffi::OsStr;
use ultramayor_engine::{
    ActiveCamera, CameraComponent, Engine, MaterialComponent, MeshComponent, TransformComponent,
    VelocityComponent,
};
use camera_controller::{apply_first_person_camera_input, FirstPersonCameraComponent};
use input_controller::{update_input_state, InputState};
use player_controller::{apply_player_input, PlayerComponent};
fn main() {
    println!("Welcome to the Game!");
    let mut engine = Engine::new();

    // Create an ECS-driven camera entity and mark it active.
    let aspect_ratio = 1024.0 / 769.0;
    let camera_transform = TransformComponent::default();

    let camera_entity = engine
        .world
        .spawn((
            camera_transform,
            CameraComponent {
                fov_y_radians: 75.0_f32.to_radians(),
                aspect_ratio,
                near: 0.1,
                far: 1000.0,
            },
            FirstPersonCameraComponent {
                yaw: -135.0,
                pitch: -45.0,
                sensitivity: 0.1,
            },
            VelocityComponent {
                translational: Vector3::new(0.0, 0.0, 0.0),
                angular: Vector3::new(0.0, 0.0, 0.0),
            },
            PlayerComponent { speed: 25.0 },
        ))
        .id();

    engine.world.insert_resource(ActiveCamera(Some(camera_entity)));
    engine.world.insert_resource(InputState::default());
    engine
        .schedule
        .add_systems((update_input_state, apply_first_person_camera_input, apply_player_input));

    let assets = [
        // engine.load_gltf(OsStr::new("resources/models/cube/Cube.gltf")),
        // engine.load_gltf(OsStr::new(
        //     "resources/models/normal_tangent_test/NormalTangentMirrorTest.gltf",
        // )),
        // engine.load_gltf(OsStr::new("resources/models/suzanne/Suzanne.gltf")),

        engine.load_gltf(OsStr::new("resources/models/avocado/Avocado.gltf")),
    ];

    let t_range = 2.0;
    for _ in 0..100 {
        for (mesh_handle, material_handle) in &assets {
            // Random position
            let pos = Vector3::new(
                random_range(-10.0..10.0),
                random_range(-10.0..10.0),
                random_range(-10.0..10.0),
            );

            // Random translational velocity
            let translational = Vector3::new(
                random_range(-t_range..t_range),
                random_range(-t_range..t_range),
                random_range(-t_range..t_range),
            );
            // let translational = Vector3::new(0.0, 0.0, 0.0);

            // Random angular velocity
            let angular = Vector3::new(
                random_range(-1.0..1.0),
                random_range(-1.0..1.0),
                random_range(-1.0..1.0),
            );

            let scale = 100.0;
            // Spawn test objects
            engine.world.spawn((
                TransformComponent {
                    position: pos,
                    rotation: nalgebra::UnitQuaternion::identity(),
                    scale: Vector3::new(scale, scale, scale),
                },
                VelocityComponent {
                    translational,
                    angular,
                },
                MeshComponent {
                    mesh_id: *mesh_handle,
                },
                MaterialComponent {
                    material_id: *material_handle,
                },
            ));
        }
    }

    engine.run();
}
