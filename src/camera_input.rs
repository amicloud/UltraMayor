// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use bevy_ecs::message::{Message, MessageReader, Messages};
use bevy_ecs::prelude::*;

use crate::handles::CameraHandle;
use crate::render_data_manager::RenderDataManager;

#[derive(Copy, Clone, Debug, Default, Resource)]
pub struct ActiveCamera(pub CameraHandle);

#[derive(Copy, Clone, Debug, Default)]
struct MouseButtons {
    left: bool,
    middle: bool,
    right: bool,
    other: bool,
    back: bool,
    forward: bool,
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

    fn any_drag(&self) -> bool {
        self.left || self.middle || self.right || self.other || self.back || self.forward
    }
}

#[derive(Copy, Clone, Debug, Default, Resource)]
pub struct CameraInputState {
    last_pos: Option<(f32, f32)>,
    buttons: MouseButtons,
}

#[derive(Copy, Clone, Debug, Message)]
pub enum CameraInputMessage {
    MouseMove { x: f32, y: f32 },
    MouseScroll { delta: f32 },
    MouseDown { button: MouseButton },
    MouseUp { button: MouseButton },
}

#[derive(Copy, Clone, Debug)]
pub enum MouseButton {
    Left,
    Middle,
    Right,
    Other,
    Back,
    Forward,
}

impl From<slint::platform::PointerEventButton> for MouseButton {
    fn from(value: slint::platform::PointerEventButton) -> Self {
        match value {
            slint::platform::PointerEventButton::Left => Self::Left,
            slint::platform::PointerEventButton::Middle => Self::Middle,
            slint::platform::PointerEventButton::Right => Self::Right,
            slint::platform::PointerEventButton::Other => Self::Other,
            slint::platform::PointerEventButton::Back => Self::Back,
            slint::platform::PointerEventButton::Forward => Self::Forward,
            _ => Self::Other,
        }
    }
}

pub fn apply_camera_input(
    mut reader: MessageReader<CameraInputMessage>,
    mut render_data_manager: ResMut<RenderDataManager>,
    active_camera: Res<ActiveCamera>,
    mut input_state: ResMut<CameraInputState>,
) {
    let camera_id = active_camera.0;
    let Some(camera) = render_data_manager.camera_manager.get_camera_mut(camera_id) else {
        for _ in reader.read() {}
        return;
    };

    for message in reader.read() {
        match *message {
            CameraInputMessage::MouseDown { button } => {
                input_state.buttons.set(button, true);
            }
            CameraInputMessage::MouseUp { button } => {
                input_state.buttons.set(button, false);
                if !input_state.buttons.any_drag() {
                    input_state.last_pos = None;
                }
            }
            CameraInputMessage::MouseMove { x, y } => {
                let delta = input_state.last_pos.map(|(lx, ly)| (x - lx, y - ly));
                input_state.last_pos = Some((x, y));

                if let Some((dx, dy)) = delta {
                    if input_state.buttons.left {
                        camera.pitch_yaw(dx, dy);
                    } else if input_state.buttons.middle {
                        camera.pan(dx, dy);
                    } else if input_state.buttons.right {
                        camera.zoom(dy);
                    }
                }
            }
            CameraInputMessage::MouseScroll { delta } => {
                camera.zoom(delta);
            }
        }
    }
}

pub fn update_camera_messages(mut messages: ResMut<Messages<CameraInputMessage>>) {
    messages.update();
}
