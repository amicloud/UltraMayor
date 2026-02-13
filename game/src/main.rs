mod bowl_controller;
mod camera_controller;
mod game_controller;
mod settings;

#[allow(unused_imports)]
use bowl_controller::{BowlFloatComponent, BowlFloatTime, update_bowl_float};
use camera_controller::{
    FlyingCameraComponent, PlayerComponent, apply_flying_camera_input,
    apply_flying_camera_movement, apply_player_movement_impulses, update_orbit_camera_target,
};
// use input_controller::{update_input_state, InputState};
use crate::camera_controller::{
    OrbitCameraComponent, apply_orbit_camera_input, apply_switch_camera_input,
    initialize_flying_camera_rotation,
};
#[allow(unused_imports)]
use crate::game_controller::{ProjectileSpawner, do_gameplay};
use bevy_ecs::schedule::IntoScheduleConfigs;
use engine::{
    ActiveCamera, CameraComponent, CollisionLayer, ConvexCollider, Engine, RenderBodyComponent,
    SleepComponent, TransformComponent, VelocityComponent,
    physics_component::{PhysicsComponent, PhysicsType},
};
use glam::{Quat, Vec3};
use rand::random_range;

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

    engine.game_schedule.add_systems(
        (
            initialize_flying_camera_rotation,
            apply_orbit_camera_input,
            update_orbit_camera_target,
            apply_flying_camera_input,
            apply_flying_camera_movement,
            apply_player_movement_impulses,
            apply_switch_camera_input,
            // do_gameplay,
            // update_bowl_float,
        )
            .chain(),
    );

    // engine.world.insert_resource(BowlFloatTime::default());

    let cube = engine
        .load_model("resources/models/cube/Cube.gltf")
        .unwrap();

    let sphere = engine
        .load_model("resources/models/sphere/sphere.obj")
        .unwrap();

    // engine.world.insert_resource(ProjectileSpawner {
    //     sphere_handle: sphere,
    //     cooldown: 0.2,
    //     cooldown_timer: 0.0,
    //     speed: 200.0,
    //     scale: 1.0,
    // });

    let player_scale: Vec3 = Vec3::splat(1.0);
    let player_start = Vec3::new(5.0, 0.0, 25.0);
    let _sphere_collider = ConvexCollider::sphere(player_scale.x, CollisionLayer::Player);
    let cuboid_collider = ConvexCollider::cuboid(player_scale * 2.0, CollisionLayer::Player);
    let _egg_collider = ConvexCollider::egg(3.0, player_scale.x, CollisionLayer::Player);
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
            drag_coefficient: 0.1,
            angular_drag_coefficient: 0.1,
            restitution: 0.5,
            local_inertia: glam::Mat3::IDENTITY,
        },
        // SleepComponent::default(),
        PlayerComponent { speed: 1.0 },
    ));

    (0..200).for_each(|i| {
        engine.world.spawn((
            TransformComponent {
                position: Vec3::new(0.0, 0.0, i as f32 * 4.01),
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
            ConvexCollider::cube(2.0, CollisionLayer::Default),
            PhysicsComponent {
                mass: 5.0,
                physics_type: PhysicsType::Dynamic,
                friction: 0.9,
                drag_coefficient: 0.1,
                angular_drag_coefficient: 0.1,
                restitution: 0.5,
                local_inertia: glam::Mat3::IDENTITY,
            },
            SleepComponent::default(),
        ));
    });

    let t_range = 2.0;

    (0..1000).for_each(|_| {
        // Random position
        let pos = Vec3::new(
            random_range(-20.0..20.0),
            random_range(-20.0..20.0),
            random_range(20.0..200.0),
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

        let scale = 1.0;
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
                render_body_id: *&sphere,
            },
            ConvexCollider::sphere(scale, CollisionLayer::Default),
            PhysicsComponent {
                mass: 3.0,
                physics_type: PhysicsType::Dynamic,
                friction: 0.2,
                drag_coefficient: 0.1,
                angular_drag_coefficient: 0.1,
                restitution: 0.5,
                local_inertia: glam::Mat3::IDENTITY,
            },
            SleepComponent::default(),
        ));
    });

    let ground = engine
        .load_model("resources/models/opalton/opalton3Dterrain.gltf")
        .unwrap();

    let ground_scale = 0.1;
    let ground_collider = engine
        .mesh_collider_from_render_body(ground, CollisionLayer::Default)
        .expect("Render body not found");
    engine.world.spawn((
        TransformComponent {
            position: Vec3::new(0.0, 0.0, -100.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(ground_scale, ground_scale, ground_scale),
        },
        RenderBodyComponent {
            render_body_id: ground,
        },
        ground_collider,
        PhysicsComponent {
            mass: f32::INFINITY,
            physics_type: PhysicsType::Static,
            friction: 0.2,
            drag_coefficient: 0.1,
            angular_drag_coefficient: 0.1,
            restitution: 0.3,
            local_inertia: glam::Mat3::IDENTITY,
        },
    ));

    // let monkey_ball_platform = engine
    //     .load_model("resources/models/platform/platform.obj")
    //     .unwrap();
    // let money_ball_collider = engine
    //     .mesh_collider_from_render_body(monkey_ball_platform, CollisionLayer::Default)
    //     .expect("Render body AABB not found");

    // engine.world.spawn((
    //     TransformComponent {
    //         position: Vec3::new(0.0, 0.0, 0.0),
    //         rotation: Quat::IDENTITY,
    //         scale: Vec3::splat(2.0),
    //     },
    //     RenderBodyComponent {
    //         render_body_id: monkey_ball_platform,
    //     },
    //     money_ball_collider,
    //     PhysicsComponent {
    //         mass: f32::INFINITY,
    //         physics_type: PhysicsType::Static,
    //         friction: 0.5,
    //         drag_coefficient: 0.1,
    //         angular_drag_coefficient: 0.1,
    //         restitution: 0.3,
    //         local_inertia: glam::Mat3::IDENTITY,
    //     },
    // ));

    // let bowl = engine.load_model("resources/models/bowl/bowl.obj").unwrap();
    // let bowl_collider = engine
    //     .mesh_collider_from_render_body(bowl, CollisionLayer::Default)
    //     .expect("Render body AABB not found");
    // engine.world.spawn((
    //     TransformComponent {
    //         position: Vec3::splat(0.0),
    //         rotation: Quat::IDENTITY,
    //         scale: Vec3::splat(0.22),
    //     },
    //     VelocityComponent {
    //         translational: Vec3::ZERO,
    //         angular: Vec3::ZERO,
    //     },
    //     RenderBodyComponent {
    //         render_body_id: bowl,
    //     },
    //     bowl_collider,
    //     BowlFloatComponent {
    //         base_height: 0.0,
    //         amplitude: 0.0,
    //         speed: 10.0,
    //     },
    //     PhysicsComponent {
    //         mass: f32::INFINITY,
    //         physics_type: PhysicsType::Static,
    //         friction: 0.5,
    //         drag_coefficient: 0.1,
    //         angular_drag_coefficient: 0.1,
    //         restitution: 0.6,
    //         local_inertia: glam::Mat3::IDENTITY,
    //     },
    // ));

    engine.run();
}
