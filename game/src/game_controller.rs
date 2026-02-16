use bevy_ecs::prelude::*;
use engine::input::InputStateResource;
use engine::
    WorldBasis
;
use sdl2::keyboard::Keycode;

pub fn do_gameplay(
    mut world: ResMut<WorldBasis>,
    input_state: Res<InputStateResource>,
) {
    let g = world.gravity_vector();
    let right = world.right();
    let up = world.up();
    let ratio = 100.0;
    if input_state.key_held(Keycode::I) {
        world.set_gravity_vector(g + (up / ratio));
    }
    if input_state.key_held(Keycode::J) {
        world.set_gravity_vector(g + (-right / ratio));
    }
    if input_state.key_held(Keycode::K) {
        world.set_gravity_vector(g + (-up / ratio));
    }
    if input_state.key_held(Keycode::L) {
        world.set_gravity_vector(g + (right / ratio));
    }
}
