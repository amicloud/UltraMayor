use bevy_ecs::prelude::*;
use engine::assets::sound_resource::SoundResource;
use engine::audio::audio_control::AudioControl;
use engine::components::single_audio_listener_component::SingleAudioListenerComponent;
use engine::input::InputStateResource;
use engine::scene::scene::Scene;
use engine::scene::scene_changer_resource::SceneChangerResource;
use engine::scene::scene_services::SceneServices;
use engine::{ActiveCamera, CameraComponent, VelocityComponent};
use engine::{Gravity, TimeResource, TransformComponent, WorldBasis};
use glam::{Quat, Vec3};
use sdl2::keyboard::Keycode;

use crate::camera_controller::*;

pub fn scene_switcher(
    mut scene_changer: ResMut<SceneChangerResource>,
    input_state: Res<InputStateResource>,
    services: Res<SceneServices>,
) {
    if input_state.key_pressed(Keycode::Tab) {
        log::info!("Tab pressed, requesting scene switch...");
        let mut new_scene = Scene::new(&services);
        make_test_scene(&mut new_scene);
        scene_changer.request_change(new_scene);
    }
}

fn make_test_scene(scene: &mut Scene) {
    let aspect_ratio = 1024.0 / 769.0;
    let _flying_camera = scene
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

    let _orbit_camera = scene
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

    scene
        .world
        .get_resource_mut::<ActiveCamera>()
        .unwrap()
        .set(_flying_camera);

    scene.game_simulation_schedule.add_systems(
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

    scene.game_frame_schedule.add_systems((
        apply_orbit_camera_input,
        apply_switch_camera_input,
        sound_control,
        scene_switcher,
        // spatial_audio_popping_demo,
    ));
}

pub fn do_gameplay(
    world: Res<WorldBasis>,
    mut gravity: ResMut<Gravity>,
    input_state: Res<InputStateResource>,
    time: Res<TimeResource>,
) {
    let right = world.right();
    let forward = world.forward();
    let ratio = time.frame_delta_time();
    let mut did_input = false;
    if input_state.key_held(Keycode::I) {
        gravity.rotate_gravity_around_axis(right, ratio);
        did_input = true;
    }
    if input_state.key_held(Keycode::J) {
        gravity.rotate_gravity_around_axis(forward, ratio);
        did_input = true;
    }
    if input_state.key_held(Keycode::K) {
        gravity.rotate_gravity_around_axis(right, -ratio);
        did_input = true;
    }
    if input_state.key_held(Keycode::L) {
        gravity.rotate_gravity_around_axis(forward, -ratio);
        did_input = true;
    }

    if did_input {
        // Clamp the angle to 20 degrees from the canonical gravity direction
        let canonical_gravity = Gravity::default().gravity_vector();
        let current_gravity = gravity.gravity_vector();
        let angle = current_gravity.angle_between(canonical_gravity);
        let max_angle = 30.0_f32.to_radians();
        if angle > max_angle {
            let axis = current_gravity.cross(canonical_gravity).normalize_or_zero();
            let rotation = Quat::from_axis_angle(axis, angle - max_angle);
            gravity.rotate_gravity(rotation);
        }
    } else {
        // Spring gravity back to normal over time
        let canonical_gravity = Gravity::default().gravity_vector();
        let gravity_diff = canonical_gravity - gravity.gravity_vector();
        let spring_strength = 5.0;
        let spring_force = gravity_diff * spring_strength * ratio;
        // If the spring force is very small, just snap back to default to avoid jitter.
        if spring_force.length() < 0.01 {
            gravity.reset();
        } else {
            gravity.gravity_normal += spring_force.normalize_or_zero() * ratio;
        }
    }
}

pub fn sound_control(
    input_state: Res<InputStateResource>,
    sound_resource: Res<SoundResource>,
    mut audio: ResMut<AudioControl>,
) {
    if input_state.key_pressed(sdl2::keyboard::Keycode::N) {
        audio.play_one_shot(
            2,
            sound_resource
                .read()
                .get_by_name("sea_shanty_2.wav")
                .unwrap(),
            0.5,
        );
    } else if input_state.key_pressed(sdl2::keyboard::Keycode::P) {
        audio.pause_mix();
    } else if input_state.key_pressed(sdl2::keyboard::Keycode::O) {
        audio.resume_mix();
    } else if input_state.key_pressed(sdl2::keyboard::Keycode::M) {
        audio.mute_mix();
    } else if input_state.key_pressed(sdl2::keyboard::Keycode::U) {
        audio.unmute_mix();
    }
}

pub fn spatial_audio_orbit_demo(
    mut query: Query<(&mut TransformComponent, &SpatialAudioDemoComponent)>,
    time: Res<TimeResource>,
) {
    for (mut transform, _) in query.iter_mut() {
        let rotation_speed = 0.5; // Radians per second
        let angle = time.total_time() * rotation_speed;
        let radius = 5.0;
        transform.position = Vec3::new(
            (angle.cos() * radius) as f32,
            (angle.sin() * radius) as f32,
            2.0,
        );
    }
}

#[derive(Component)]
pub struct SpatialAudioDemoComponent;

// #[allow(unused)]
// pub fn spatial_audio_popping_demo(
//     time: Res<TimeResource>,
//     mut audio_command_queue: ResMut<AudioCommandQueue>,
//     sound_resource: Res<SoundResource>,
// ) {
//     if time.frame_count().is_multiple_of(120) {
//         eprint!("POP!");
//         let position = Vec3::new(5.0, 0.0, 2.0);
//         audio_command_queue.push(AudioCommand::PlaySoundAtLocation {
//             track: 0,
//             sound: sound_resource.read().get_by_name("pop.wav").unwrap(),
//             volume: 2.0,
//             looping: false,
//             location: position,
//         });
//     }
// }
