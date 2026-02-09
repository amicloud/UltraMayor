use bevy_ecs::component::Component;
use glam::{Mat4, Quat, Vec3};

#[derive(Component, Debug, Clone, Copy)]
pub struct TransformComponent {
    pub position: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,
}

impl Default for TransformComponent {
    fn default() -> Self {
        Self {
            position: Vec3::new(0.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(1.0, 1.0, 1.0),
        }
    }
}

impl TransformComponent {
    pub fn to_mat4(&self) -> Mat4 {
        let translation_matrix = Mat4::from_translation(self.position);
        let rotation_matrix = Mat4::from_quat(self.rotation);
        let scale_matrix = Mat4::from_scale(self.scale);

        translation_matrix * rotation_matrix * scale_matrix
    }
}
