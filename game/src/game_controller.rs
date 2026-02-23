use bevy_ecs::prelude::*;
use engine::assets::sound_resource::SoundResource;
use engine::audio::command_queue::{AudioCommand, AudioCommandQueue};
use engine::input::InputStateResource;
use engine::{Gravity, TimeResource, WorldBasis};
use glam::{Quat, Vec3};
use sdl2::keyboard::Keycode;

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
    mut audio_command_queue: ResMut<AudioCommandQueue>,
    sound_resource: Res<SoundResource>,
) {
    if input_state.key_pressed(sdl2::keyboard::Keycode::N) {
        audio_command_queue.push(AudioCommand::PlaySound {
            track: 0,
            sound: sound_resource.get_by_name("sea_shanty_2.wav").unwrap(),
            volume: 0.5,
            looping: false,
            location: Some(Vec3::splat(0.0)),
            source: None,
        });
    } else if input_state.key_pressed(sdl2::keyboard::Keycode::P) {
        audio_command_queue.push(AudioCommand::PauseMix);
    } else if input_state.key_pressed(sdl2::keyboard::Keycode::O) {
        audio_command_queue.push(AudioCommand::ResumeMix);
    } else if input_state.key_pressed(sdl2::keyboard::Keycode::M) {
        audio_command_queue.push(AudioCommand::MuteMix);
    } else if input_state.key_pressed(sdl2::keyboard::Keycode::U) {
        audio_command_queue.push(AudioCommand::UnmuteMix);
    }
}
