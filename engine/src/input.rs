// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use std::collections::HashSet;

use bevy_ecs::resource::Resource;
use sdl2::keyboard::Keycode;

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum MouseButton {
    Left,
    Middle,
    Right,
    Other,
    Back,
    Forward,
}

impl From<sdl2::mouse::MouseButton> for MouseButton {
    fn from(button: sdl2::mouse::MouseButton) -> Self {
        match button {
            sdl2::mouse::MouseButton::Left => MouseButton::Left,
            sdl2::mouse::MouseButton::Middle => MouseButton::Middle,
            sdl2::mouse::MouseButton::Right => MouseButton::Right,
            sdl2::mouse::MouseButton::X1 => MouseButton::Back,
            sdl2::mouse::MouseButton::X2 => MouseButton::Forward,
            _ => MouseButton::Other,
        }
    }
}

#[derive(Default, Resource)]
pub struct InputStateResource {
    pub(crate) current_keys: HashSet<Keycode>,
    pub(crate) previous_keys: HashSet<Keycode>,

    pub mouse_delta: (f32, f32),
    pub scroll_delta: f32,
    pub current_mouse_buttons: HashSet<MouseButton>,
    pub previous_mouse_buttons: HashSet<MouseButton>,
}

impl InputStateResource {
    pub fn key_held(&self, key: Keycode) -> bool {
        self.current_keys.contains(&key)
    }

    pub fn key_pressed(&self, key: Keycode) -> bool {
        self.current_keys.contains(&key) && !self.previous_keys.contains(&key)
    }

    pub fn key_released(&self, key: Keycode) -> bool {
        !self.current_keys.contains(&key) && self.previous_keys.contains(&key)
    }

    pub fn mouse_button_held(&self, button: MouseButton) -> bool {
        self.current_mouse_buttons.contains(&button)
    }

    pub fn mouse_button_pressed(&self, button: MouseButton) -> bool {
        self.current_mouse_buttons.contains(&button)
            && !self.previous_mouse_buttons.contains(&button)
    }

    pub fn mouse_button_released(&self, button: MouseButton) -> bool {
        !self.current_mouse_buttons.contains(&button)
            && self.previous_mouse_buttons.contains(&button)
    }
}
