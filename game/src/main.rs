mod settings;

use nalgebra::Vector3;
use rand::random_range;
use std::ffi::OsStr;
use ultramayor_engine::{Engine, MaterialComponent, MeshComponent, TransformComponent, VelocityComponent};
fn main() {
    println!("Welcome to the Game!");
    let mut engine = Engine::new();

    let assets = [
        // engine.load_gltf(OsStr::new("resources/models/cube/Cube.gltf")),
        // engine.load_gltf(OsStr::new(
        //     "resources/models/normal_tangent_test/NormalTangentMirrorTest.gltf",
        // )),
        // engine.load_gltf(OsStr::new("resources/models/suzanne/Suzanne.gltf")),

        engine.load_gltf(OsStr::new("resources/models/acvodad/Avocado.gltf")),
    ];

    let t_range = 2.0;
    for _ in 0..1000 {
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
