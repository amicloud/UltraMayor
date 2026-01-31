// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use bevy_ecs::prelude::Resource;
use nalgebra::Vector3;

/// Canonical world coordinate basis.
///
/// Right-handed, Z-up, -Y forward.
#[derive(Resource, Debug, Copy, Clone)]
pub struct WorldBasis {
    up: Vector3<f32>,
    forward: Vector3<f32>,
    right: Vector3<f32>,
}

impl WorldBasis {
    pub fn new(up: Vector3<f32>, forward: Vector3<f32>) -> Self {
        let up = up.normalize();
        let forward = forward.normalize();
        let right = forward.cross(&up).normalize();

        Self { up, forward, right }
    }

    pub fn canonical() -> Self {
        Self::new(Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, -1.0, 0.0))
    }

    pub fn up(&self) -> Vector3<f32> {
        self.up
    }

    pub fn forward(&self) -> Vector3<f32> {
        self.forward
    }

    pub fn right(&self) -> Vector3<f32> {
        self.right
    }
}

impl Default for WorldBasis {
    fn default() -> Self {
        Self::canonical()
    }
}
