use bevy_ecs::message::MessageReader;
use bevy_ecs::prelude::*;
use sdl2::keyboard::Keycode;
use ultramayor_engine::{InputMessage, MouseButton};

/// Mouse buttons tracked by the shared input controller.
#[derive(Copy, Clone, Debug, Default)]
pub struct MouseButtons {
    pub left: bool,
    pub middle: bool,
    pub right: bool,
    pub other: bool,
    pub back: bool,
    pub forward: bool,
}

impl MouseButtons {
    fn set(&mut self, button: MouseButton, pressed: bool) {
        match button {
            MouseButton::Left => self.left = pressed,
            MouseButton::Middle => self.middle = pressed,
            MouseButton::Right => self.right = pressed,
            MouseButton::Other => self.other = pressed,
            MouseButton::Back => self.back = pressed,
            MouseButton::Forward => self.forward = pressed,
        }
    }

    pub fn any(&self) -> bool {
        self.left || self.middle || self.right || self.other || self.back || self.forward
    }
}

/// Key buttons tracked by the shared input controller.
#[derive(Copy, Clone, Debug, Default)]
pub struct KeyButtons {
    pub forward: bool,
    pub back: bool,
    pub left: bool,
    pub right: bool,
    pub up: bool,
    pub down: bool,
}

impl KeyButtons {
    fn set(&mut self, keycode: Keycode, state: bool) {
        match keycode {
            Keycode::W => self.forward = state,
            Keycode::S => self.back = state,
            Keycode::A => self.left = state,
            Keycode::D => self.right = state,
            Keycode::Space => self.up = state,
            Keycode::LShift | Keycode::RShift => self.down = state,
            _ => {}
        }
    }
}

/// Shared input state updated from engine `InputMessage`s.
#[derive(Copy, Clone, Debug, Default, Resource)]
pub struct InputState {
    pub mouse_delta: (f32, f32),
    pub scroll_delta: f32,
    pub mouse_buttons: MouseButtons,
    pub keys: KeyButtons,
    last_pos: Option<(f32, f32)>,
}

/// Updates the shared input state from engine messages.
pub fn update_input_state(
    mut reader: MessageReader<InputMessage>,
    mut input_state: ResMut<InputState>,
) {
    input_state.mouse_delta = (0.0, 0.0);
    input_state.scroll_delta = 0.0;

    for message in reader.read() {
        match *message {
            InputMessage::MouseButtonDown { button } => {
                input_state.mouse_buttons.set(button, true);
            }
            InputMessage::MouseButtonUp { button } => {
                input_state.mouse_buttons.set(button, false);
                if !input_state.mouse_buttons.any() {
                    input_state.last_pos = None;
                }
            }
            InputMessage::MouseMove { x, y } => {
                let delta = input_state.last_pos.map(|(lx, ly)| (x - lx, y - ly));
                input_state.last_pos = Some((x, y));
                if let Some((dx, dy)) = delta {
                    input_state.mouse_delta.0 += dx;
                    input_state.mouse_delta.1 += dy;
                }
            }
            InputMessage::MouseScroll { delta } => {
                input_state.scroll_delta += delta;
            }
            InputMessage::KeyDown { keycode } => {
                input_state.keys.set(keycode, true);
            }
            InputMessage::KeyUp { keycode } => {
                input_state.keys.set(keycode, false);
            }
        }
    }
}
