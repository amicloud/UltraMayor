mod camera_controller;
mod game_controller;
mod settings;

#[allow(unused_imports)]
use camera_controller::{
    FlyingCameraComponent, OrbitCameraComponent, PlayerComponent, apply_flying_camera_input,
    apply_flying_camera_movement, apply_orbit_camera_input, apply_player_movement_impulses,
    apply_switch_camera_input, initialize_flying_camera_rotation, update_orbit_camera_target,
};
use engine::components::single_audio_listener_component::SingleAudioListenerComponent;
#[allow(unused_imports)]
use engine::components::{
    audio_source_component::AudioSourceComponent,
    physics_event_listener_component::PhysicsEventListenerComponent,
};

use crate::game_controller::{
    SpatialAudioDemoComponent, do_gameplay, sound_control, spatial_audio_orbit_demo,
};
use bevy_ecs::schedule::IntoScheduleConfigs;
use engine::{
    ActiveCamera, CameraComponent, CollisionLayer, ConvexCollider, Engine, RenderBodyComponent,
    SleepComponent, TransformComponent, VelocityComponent,
};

use engine::components::physics_component::{PhysicsComponent, PhysicsType};

use glam::{Quat, Vec3};

#[cfg(feature = "dhat-heap")]
#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

fn main() {
    #[cfg(feature = "dhat-heap")]
    let _profiler = dhat::Profiler::new_heap();

    println!("Welcome to the Game!");
    let mut engine = Engine::new();

    // Create an ECS-driven camera entity and mark it active.
    let aspect_ratio = 1024.0 / 769.0;

    let _flying_camera = engine
        .world
        .spawn((
            SingleAudioListenerComponent,
            TransformComponent {
                position: Vec3::new(0.0, 0.0, 2.0),
                rotation: Quat::IDENTITY,
                scale: Vec3::new(1.0, 1.0, 1.0),
            },
            CameraComponent {
                fov_y_radians: 75.0_f32.to_radians(),
                aspect_ratio,
                near: 0.1,
                far: 10000.0,
            },
            FlyingCameraComponent {
                yaw: -135.0,
                pitch: 0.0,
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
            TransformComponent {
                position: Vec3::new(0.0, 0.0, 0.0),
                rotation: Quat::IDENTITY,
                scale: Vec3::new(1.0, 1.0, 1.0),
            },
            CameraComponent {
                fov_y_radians: 75.0_f32.to_radians(),
                aspect_ratio,
                near: 0.1,
                far: 10000.0,
            },
            OrbitCameraComponent {
                target: Vec3::new(0.0, 0.0, 0.0),
                distance: 100.0,
                yaw: 0.0,
                pitch: -30.0,
                sensitivity: 0.2,
            },
        ))
        .id();

    engine
        .world
        .get_resource_mut::<ActiveCamera>()
        .unwrap()
        .set(_flying_camera);

    engine.game_simulation_schedule.add_systems(
        (
            initialize_flying_camera_rotation,
            apply_orbit_camera_input,
            update_orbit_camera_target,
            apply_flying_camera_input,
            apply_flying_camera_movement,
            apply_player_movement_impulses,
            do_gameplay,
            spatial_audio_orbit_demo,
            // update_bowl_float,
        )
            .chain(),
    );

    engine.game_frame_schedule.add_systems((
        apply_orbit_camera_input,
        apply_switch_camera_input,
        sound_control,
    ));

    let cube = engine
        .load_model("resources/models/cube/Cube.gltf")
        .unwrap();

    let _sphere = engine
        .load_model("resources/models/sphere_low/sphere.obj")
        .unwrap();

    let player_scale: Vec3 = Vec3::splat(1.0);
    let player_start = Vec3::new(0.0, 5.0, 2.2);
    let player_local_aabb = engine
        .aabb_from_render_body(cube)
        .expect("Player render body AABB not found");
    let player_local_size = player_local_aabb.max - player_local_aabb.min;
    let _sphere_collider = ConvexCollider::sphere(
        player_local_size.max_element() * 0.5,
        CollisionLayer::Player,
    );
    let cuboid_collider = ConvexCollider::cuboid(player_local_size, CollisionLayer::Player);
    let _egg_collider = ConvexCollider::egg(3.0, player_scale.x, CollisionLayer::Player);

    let _sea_shanty = engine
        .load_wav("resources/sounds/sea_shanty_2.wav")
        .expect("Failed to load sound");

    let _sort_shanty = engine
        .load_wav("resources/sounds/sea_shanty_2_short.wav")
        .expect("Failed to load sound");
    engine.world.spawn((
        TransformComponent {
            position: player_start,
            rotation: Quat::IDENTITY,
            scale: player_scale,
        },
        VelocityComponent {
            translational: Vec3::ZERO,
            angular: Vec3::ZERO,
        },
        RenderBodyComponent {
            render_body_id: cube,
        },
        cuboid_collider,
        PhysicsComponent {
            mass: 5.0,
            physics_type: PhysicsType::Dynamic,
            friction: 0.9,
            drag_coefficient: 0.8,
            angular_drag_coefficient: 0.1,
            restitution: 0.5,
            local_inertia: glam::Mat3::IDENTITY,
        },
        // SleepComponent::default(),
        PlayerComponent { speed: 1.0 },
        PhysicsEventListenerComponent {},
    ));

    // Spatial audio testing
    engine.world.spawn((
        TransformComponent {
            position: Vec3::new(0.0, 0.0, 5.0),
            rotation: Quat::IDENTITY,
            scale: player_scale,
        },
        VelocityComponent {
            translational: Vec3::ZERO,
            angular: Vec3::ZERO,
        },
        RenderBodyComponent {
            render_body_id: cube,
        },
        cuboid_collider,
        SleepComponent::default(),
        AudioSourceComponent {
            sound: _sea_shanty,
            volume: 1.0,
            looping: true,
            pitch: 1.0,
        },
        SpatialAudioDemoComponent,
    ));

    // #[cfg(debug_assertions)]
    // let spawn_mult = 1;

    // #[cfg(not(debug_assertions))]
    // let spawn_mult = 10;

    // (1..=(5 * spawn_mult)).for_each(|i| {
    //     let p = player_local_size * Vec3::new(0.0, 0.0, i as f32);
    //     engine.world.spawn((
    //         TransformComponent {
    //             position: p,
    //             rotation: Quat::IDENTITY,
    //             scale: player_scale,
    //         },
    //         VelocityComponent {
    //             translational: Vec3::ZERO,
    //             angular: Vec3::ZERO,
    //         },
    //         RenderBodyComponent {
    //             render_body_id: cube,
    //         },
    //         cuboid_collider,
    //         PhysicsComponent {
    //             mass: 5.0,
    //             physics_type: if i == 1 {
    //                 PhysicsType::Static
    //             } else {
    //                 PhysicsType::Dynamic
    //             },
    //             friction: 0.9,
    //             drag_coefficient: 0.1,
    //             angular_drag_coefficient: 0.1,
    //             restitution: 0.5,
    //             local_inertia: glam::Mat3::IDENTITY,
    //         },
    //         SleepComponent::default(),
    //     ));
    // });

    // let t_range = 2.0;

    // (0..(10 * spawn_mult)).for_each(|_| {
    //     use rand::random_range;
    //     // Random position
    //     let pos = Vec3::new(
    //         random_range(-20.0..20.0),
    //         random_range(-20.0..20.0),
    //         random_range(20.0..200.0),
    //     );

    //     // Random translational velocity
    //     let translational = Vec3::new(
    //         random_range((-t_range)..t_range),
    //         random_range((-t_range)..t_range),
    //         random_range((-t_range)..t_range),
    //     );
    //     // let translational = Vec3::new(0.0, 0.0, 0.0);

    //     // Random angular velocity
    //     let angular = Vec3::new(
    //         random_range(-1.0..1.0),
    //         random_range(-1.0..1.0),
    //         random_range(-1.0..1.0),
    //     );

    //     let scale = 1.0;
    //     // Spawn test objects
    //     engine.world.spawn((
    //         TransformComponent {
    //             position: pos,
    //             rotation: Quat::IDENTITY,
    //             scale: Vec3::new(scale, scale, scale),
    //         },
    //         VelocityComponent {
    //             translational,
    //             angular,
    //         },
    //         RenderBodyComponent {
    //             render_body_id: sphere,
    //         },
    //         ConvexCollider::sphere(scale, CollisionLayer::Default),
    //         PhysicsComponent {
    //             mass: 300.0,
    //             physics_type: PhysicsType::Dynamic,
    //             friction: 0.2,
    //             drag_coefficient: 0.1,
    //             angular_drag_coefficient: 0.1,
    //             restitution: 0.5,
    //             local_inertia: glam::Mat3::IDENTITY,
    //         },
    //         SleepComponent::default(),
    //     ));
    // });

    // let test_ground = engine
    //     .load_model("resources/models/test_ground/test_ground.obj")
    //     .unwrap();
    // let test_ground_collider = engine
    //     .mesh_collider_from_render_body(test_ground, CollisionLayer::Default)
    //     .expect("Render body AABB not found");
    // engine.world.spawn((
    //     TransformComponent {
    //         position: Vec3::new(0.0, 0.0, 0.0),
    //         rotation: Quat::IDENTITY,
    //         scale: Vec3::splat(10.0),
    //     },
    //     RenderBodyComponent {
    //         render_body_id: test_ground,
    //     },
    //     test_ground_collider,
    //     PhysicsComponent {
    //         mass: f32::INFINITY,
    //         physics_type: PhysicsType::Static,
    //         friction: 0.2,
    //         drag_coefficient: 0.1,
    //         angular_drag_coefficient: 0.1,
    //         restitution: 0.3,
    //         local_inertia: glam::Mat3::IDENTITY,
    //     },
    // ));

    let platform = engine
        .load_model("resources/models/platform/platform.obj")
        .unwrap();
    let platform_mesh_collider = engine
        .mesh_collider_from_render_body(platform, CollisionLayer::Default)
        .expect("Render body AABB not found");

    engine.world.spawn((
        TransformComponent {
            position: Vec3::new(0.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::splat(2.0),
        },
        RenderBodyComponent {
            render_body_id: platform,
        },
        platform_mesh_collider,
        PhysicsComponent {
            mass: f32::INFINITY,
            physics_type: PhysicsType::Static,
            friction: 0.5,
            drag_coefficient: 0.1,
            angular_drag_coefficient: 0.1,
            restitution: 0.3,
            local_inertia: glam::Mat3::IDENTITY,
        },
    ));
    // engine.world.add_observer(|collision: On<PhysicsEvent>| {
    //     if collision.event_type == PhysicsEventType::Hit {
    //         engine.world.get_resource_mut::<AudioQueue>().unwrap().instances.push(AudioInstance {
    //             sound: sound,
    //             volume: 0.5,
    //             looping: false,
    //         });
    //     }
    // });
    engine.run();
}
