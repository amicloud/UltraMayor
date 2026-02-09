// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use bevy_ecs::prelude::*;
use glam::Mat4;

use crate::transform_component::TransformComponent;

/// Camera projection data owned by game logic.
///
/// Note: This component intentionally does NOT store any transform data.
/// A camera entity must also have a `TransformComponent` to provide view data.
#[derive(Component, Debug, Clone, Copy)]
#[require(TransformComponent)]
pub struct CameraComponent {
    /// Vertical field-of-view in radians.
    pub fov_y_radians: f32,
    pub aspect_ratio: f32,
    pub near: f32,
    pub far: f32,
}

impl CameraComponent {
    pub fn projection_matrix(&self) -> Mat4 {
        Mat4::perspective_rh(self.fov_y_radians, self.aspect_ratio, self.near, self.far)
    }
}

/// The active camera entity used for rendering.
///
/// This is optional so games can decide when a camera becomes active.
#[derive(Resource, Debug, Default, Clone, Copy)]
pub struct ActiveCamera(pub Option<Entity>);

impl ActiveCamera {
    pub fn get(&self) -> Option<Entity> {
        self.0
    }

    /// Sets the active camera entity.
    pub fn set(&mut self, entity: Entity) {
        self.0 = Some(entity);
    }
}
