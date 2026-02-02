mod camera_controller;
mod input_controller;
mod settings;

use camera_controller::{
    apply_flying_camera_input, apply_flying_camera_movement, FlyingCameraComponent,
};
// use input_controller::{update_input_state, InputState};
use engine::{
    physics_component::{PhysicsComponent, PhysicsType},
    physics_resource::Impulse,
    ActiveCamera, CameraComponent, CollisionLayer, Engine, RenderBodyComponent, TransformComponent,
    VelocityComponent
};
use glam::{Quat, Vec3};
use rand::random_range;

use crate::camera_controller::{
    apply_orbit_camera_input, apply_switch_camera_input, initialize_flying_camera_rotation,
    OrbitCameraComponent,
};
fn main() {
    println!("Welcome to the Game!");
    let mut engine = Engine::new();

    // Create an ECS-driven camera entity and mark it active.
    let aspect_ratio = 1024.0 / 769.0;
    let camera_transform = TransformComponent {
        position: Vec3::new(0.0, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        scale: Vec3::new(1.0, 1.0, 1.0),
    };

    let flying_camera = engine
        .world
        .spawn((
            camera_transform,
            CameraComponent {
                fov_y_radians: 75.0_f32.to_radians(),
                aspect_ratio,
                near: 0.1,
                far: 10000.0,
            },
            FlyingCameraComponent {
                yaw: -135.0,
                pitch: -45.0,
                sensitivity: 0.1,
                speed: 100.0
            },
            VelocityComponent {
                translational: Vec3::new(0.0, 0.0, 0.0),
                angular: Vec3::new(0.0, 0.0, 0.0),
            },
        ))
        .id();

    #[allow(unused_variables)]
    let orbit_camera = engine
        .world
        .spawn((
            camera_transform,
            CameraComponent {
                fov_y_radians: 75.0_f32.to_radians(),
                aspect_ratio,
                near: 0.1,
                far: 10000.0,
            },
            OrbitCameraComponent {
                target: Vec3::new(0.0, 0.0, 0.0),
                distance: 100.0,
                yaw: -135.0,
                pitch: -30.0,
                sensitivity: 0.2,
            },
        ))
        .id();

    engine
        .world
        .get_resource_mut::<ActiveCamera>()
        .unwrap()
        .set(flying_camera);

    engine.schedule.add_systems((
        initialize_flying_camera_rotation,
        apply_orbit_camera_input,
        apply_flying_camera_input,
        apply_flying_camera_movement,
        apply_switch_camera_input,
    ));

    let assets = [
        // engine.load_gltf(OsStr::new("resources/models/cube/Cube.gltf")),
        // engine.load_gltf(OsStr::new(
        //     "resources/models/normal_tangent_test/NormalTangentMirrorTest.gltf",
        // )),
        // engine.load_gltf(OsStr::new("resources/models/suzanne/Suzanne.gltf")),
        // engine.load_model("resources/models/avocado/Avocado.gltf").unwrap(),
        engine
            .load_model("resources/models/building/building.obj")
            .unwrap(),
    ];

    let ground = engine
        .load_model("resources/models/opalton/opalton3Dterrain.gltf")
        .unwrap();

    let antique_camera = engine
        .load_model("resources/models/antique_camera/AntiqueCamera.gltf")
        .unwrap();

    let t_range = 2.0;
    for _ in 0..1 {
        for render_body_handle in &assets {
            // Random position
            let pos = Vec3::new(
                random_range(-10.0..10.0),
                random_range(-10.0..10.0),
                random_range(300.0..325.0),
            );

            // Random translational velocity
            let translational = Vec3::new(
                random_range(-t_range..t_range),
                random_range(-t_range..t_range),
                random_range(-t_range..t_range),
            );
            // let translational = Vec3::new(0.0, 0.0, 0.0);

            // Random angular velocity
            let angular = Vec3::new(
                random_range(-1.0..1.0),
                random_range(-1.0..1.0),
                random_range(-1.0..1.0),
            );

            let scale = 100.0;
            let collider = engine
                .collider_from_render_body(*render_body_handle, CollisionLayer::Default)
                .expect("Render body AABB not found");
            // Spawn test objects
            engine.world.spawn((
                TransformComponent {
                    position: pos,
                    rotation: Quat::IDENTITY,
                    scale: Vec3::new(scale, scale, scale),
                },
                VelocityComponent {
                    translational,
                    angular,
                },
                RenderBodyComponent {
                    render_body_id: *render_body_handle,
                },
                collider,
            ));
        }
    }

    let antique_camera_scale = 10.0;
    let mut antique_camera_transform = TransformComponent {
        position: camera_transform.position,
        rotation: Quat::IDENTITY,
        scale: Vec3::new(
            antique_camera_scale,
            antique_camera_scale,
            antique_camera_scale,
        ),
    };

    let antique_camera_velocity = VelocityComponent {
        translational: Vec3::new(0.0, 0.0, 0.0),
        angular: Vec3::new(0.0, 1.0, 0.0),
    };

    antique_camera_transform.position.x += 50.0;
    antique_camera_transform.position.y += 50.0;

    engine.world.spawn((
        antique_camera_transform,
        RenderBodyComponent {
            render_body_id: antique_camera,
        },
        antique_camera_velocity,
    ));
    engine.world.spawn((
        antique_camera_transform,
        RenderBodyComponent {
            render_body_id: antique_camera,
        },
        antique_camera_velocity / 2.0,
    ));
    let antique_collider = engine
        .collider_from_render_body(antique_camera, CollisionLayer::Default)
        .expect("Render body AABB not found");
    let phys_cam = engine
        .world
        .spawn((
            antique_camera_transform,
            RenderBodyComponent {
                render_body_id: antique_camera,
            },
            antique_camera_velocity / 3.0,
            PhysicsComponent {
                mass: 1.0,
                physics_type: PhysicsType::Dynamic,
                friction: 0.5,
                drag_coefficient: 0.1,
                angular_drag_coefficient: 0.1,
                restitution: 0.5,
            },
            antique_collider,
        ))
        .id();

    engine.add_impulse(Impulse {
        entity: phys_cam,
        linear: Vec3::new(5.0, 2.0, 100.0) * 1.0,
        angular: Vec3::new(0.0, 0.0, 0.0),
    });

    let ground_scale = 1.0;
    let ground_collider = engine
        .collider_from_render_body(ground, CollisionLayer::Default)
        .expect("Render body AABB not found");
    engine.world.spawn((
        TransformComponent {
            position: Vec3::new(0.0, 0.0, -300.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(ground_scale, ground_scale, 1.0),
        },
        RenderBodyComponent {
            render_body_id: ground,
        },
        ground_collider,
    ));

    engine.run();
}
