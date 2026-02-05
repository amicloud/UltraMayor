mod camera_controller;
mod input_controller;
mod settings;

use camera_controller::{
    apply_flying_camera_input, apply_flying_camera_movement, apply_player_movement_impulses,
    update_orbit_camera_target, FlyingCameraComponent, PlayerComponent,
};
// use input_controller::{update_input_state, InputState};
use crate::camera_controller::{
    apply_orbit_camera_input, apply_switch_camera_input, initialize_flying_camera_rotation,
    OrbitCameraComponent,
};
use bevy_ecs::schedule::IntoScheduleConfigs;
use engine::{
    physics_component::{PhysicsComponent, PhysicsType},
    ActiveCamera, CameraComponent, CollisionLayer, ConvexCollider, Engine, RenderBodyComponent,
    SleepComponent, TransformComponent, VelocityComponent,
};
use glam::{Quat, Vec3};
use rand::random_range;
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

    let _flying_camera = engine
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
                speed: 100.0,
            },
            VelocityComponent {
                translational: Vec3::new(0.0, 0.0, 0.0),
                angular: Vec3::new(0.0, 0.0, 0.0),
            },
        ))
        .id();

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
        .set(orbit_camera);

    engine.schedule.add_systems(
        (
            initialize_flying_camera_rotation,
            apply_orbit_camera_input,
            update_orbit_camera_target,
            apply_flying_camera_input,
            apply_flying_camera_movement,
            apply_player_movement_impulses,
            apply_switch_camera_input,
        )
            .chain(),
    );

    let assets = [
        // engine.load_gltf(OsStr::new("resources/models/cube/Cube.gltf")),
        // engine.load_gltf(OsStr::new(
        //     "resources/models/normal_tangent_test/NormalTangentMirrorTest.gltf",
        // )),
        // engine.load_gltf(OsStr::new("resources/models/suzanne/Suzanne.gltf")),
        engine
            .load_model("resources/models/avocado/Avocado.gltf")
            .unwrap(),
        // engine
        //     .load_model("resources/models/building/building.obj")
        //     .unwrap(),
    ];

    let player_render_body = engine
        .load_model("resources/models/sphere/sphere.obj")
        .unwrap();

    let player_scale = 1.0;
    let player_start = Vec3::new(0.0, 0.0, 50.0);
    let player_collider = ConvexCollider::cube(1.0, CollisionLayer::Player);
    engine.world.spawn((
        TransformComponent {
            position: player_start,
            rotation: Quat::IDENTITY,
            scale: Vec3::new(player_scale, player_scale, player_scale),
        },
        VelocityComponent {
            translational: Vec3::ZERO,
            angular: Vec3::ZERO,
        },
        RenderBodyComponent {
            render_body_id: player_render_body,
        },
        player_collider,
        PhysicsComponent {
            mass: 1.0,
            physics_type: PhysicsType::Dynamic,
            friction: 0.5,
            drag_coefficient: 0.1,
            angular_drag_coefficient: 0.1,
            restitution: 0.2,
        },
        // SleepComponent::default(),
        PlayerComponent { speed: 1.0 },
    ));

    let ground = engine
        .load_model("resources/models/opalton/opalton3Dterrain.gltf")
        .unwrap();

    let antique_camera = engine
        .load_model("resources/models/antique_camera/AntiqueCamera.gltf")
        .unwrap();

    let t_range = 2.0;

    for _ in 0..100 {
        for render_body_handle in &assets {
            // Random position
            let pos = Vec3::new(
                random_range(10.0..30.0),
                random_range(-10.0..10.0),
                random_range(50.0..100.0),
            );

            // Random translational velocity
            let translational = Vec3::new(
                random_range((-t_range)..t_range),
                random_range((-t_range)..t_range),
                random_range((-t_range)..t_range),
            );
            // let translational = Vec3::new(0.0, 0.0, 0.0);

            // Random angular velocity
            let angular = Vec3::new(
                random_range(-1.0..1.0),
                random_range(-1.0..1.0),
                random_range(-1.0..1.0),
            );

            let scale = 10.0;
            let collider = engine
                .aabb_from_render_body(*render_body_handle)
                .map(|aabb| ConvexCollider::sphere_from_aabb(aabb, CollisionLayer::Default))
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
                PhysicsComponent {
                    mass: 1.0,
                    physics_type: PhysicsType::Dynamic,
                    friction: 0.5,
                    drag_coefficient: 0.1,
                    angular_drag_coefficient: 0.1,
                    restitution: 0.3,
                },
                SleepComponent::default(),
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

    let antique_collider = engine
        .aabb_from_render_body(antique_camera)
        .map(|aabb| ConvexCollider::cuboid(aabb, CollisionLayer::Default))
        .expect("Render body AABB not found");
    // let phys_cam = engine
    //     .world
    //     .spawn((
    //         antique_camera_transform,
    //         RenderBodyComponent {
    //             render_body_id: antique_camera,
    //         },
    //         antique_camera_velocity / 3.0,
    //         PhysicsComponent {
    //             mass: 10.0,
    //             physics_type: PhysicsType::Dynamic,
    //             friction: 0.5,
    //             drag_coefficient: 0.1,
    //             angular_drag_coefficient: 0.1,
    //             restitution: 0.5,
    //         },
    //         antique_collider,
    //         SleepComponent::default(),
    //     ))
    //     .id();

    // engine.add_impulse(Impulse {
    //     entity: phys_cam,
    //     linear: Vec3::new(5.0, 2.0, 100.0) * 10.0,
    //     angular: Vec3::new(0.0, 0.0, 0.0),
    // });

    let ground_scale = 0.1;
    let ground_collider = engine
        .mesh_collider_from_render_body(ground, CollisionLayer::Default)
        .expect("Render body not found");
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
        PhysicsComponent {
            mass:f32::INFINITY,
            physics_type: PhysicsType::Static,
            friction: 0.5,
            drag_coefficient: 0.1,
            angular_drag_coefficient: 0.1,
            restitution: 0.3,
        },
    ));

    // let monkey_ball_platform = engine.load_model("resources/models/platform/platform.obj");
    // let money_ball_collider = engine.mesh_collider_from_render_body(monkey_ball_platform.unwrap(), CollisionLayer::Default)
    //     .expect("Render body AABB not found");
    // engine.world.spawn((
    //     TransformComponent {
    //         position: Vec3::new(20.0, 0.0, 0.0),
    //         rotation: Quat::IDENTITY,
    //         scale: Vec3::new(10.0,10.0,10.0),
    //     },
    //     RenderBodyComponent {
    //         render_body_id: monkey_ball_platform.unwrap(),
    //     },
    //     money_ball_collider,
    // ));
    engine.run();
}
