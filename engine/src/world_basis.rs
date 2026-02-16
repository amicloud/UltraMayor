// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use bevy_ecs::prelude::Resource;
use glam::Vec3;

/// Canonical world coordinate basis.
///
/// Right-handed, Z-up, -Y forward.
#[derive(Resource, Debug, Clone, Copy)]
pub struct WorldBasis {
    up: Vec3,
    forward: Vec3,
    right: Vec3,
    gravity: Vec3,
}

impl WorldBasis {
    pub fn new(up: Vec3, forward: Vec3) -> Self {
        let up = up.normalize();
        let forward = forward.normalize();
        let right = forward.cross(up).normalize();
        let gravity = -up * 9.81;

        Self { up, forward, right, gravity }
    }

    pub fn canonical() -> Self {
        Self::new(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, -1.0, 0.0))
    }

    pub fn up(&self) -> Vec3 {
        self.up
    }

    pub fn forward(&self) -> Vec3 {
        self.forward
    }

    pub fn right(&self) -> Vec3 {
        self.right
    }

    pub fn gravity_vector(&self) -> Vec3 {
        self.gravity
    }

    pub fn set_gravity_vector(&mut self, gravity: Vec3) {
        self.gravity = gravity;
    }

    pub(crate) fn normal_gravity() -> Vec3 {
        -Self::canonical().up * Self::canonical().gravity.length()
    }
}

impl Default for WorldBasis {
    fn default() -> Self {
        Self::canonical()
    }
}
