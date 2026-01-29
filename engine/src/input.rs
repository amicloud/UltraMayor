// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

#[derive(Copy, Clone, Debug)]
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
