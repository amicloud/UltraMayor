use bevy_ecs::prelude::*;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use ultramayor_engine::{ActiveCamera, TransformComponent};

use crate::input_controller::InputState;

/// Orbit-style camera parameters controlled by input.
#[derive(Component, Debug)]
#[require(TransformComponent)]
pub struct OrbitCameraComponent {
    pub target: Vector3<f32>,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub sensitivity: f32,
}

impl OrbitCameraComponent {
    /// Updates a transform to match this orbit camera state.
    pub fn apply_to_transform(&self, transform: &mut TransformComponent) {
        let yaw_rad = self.yaw.to_radians();
        let pitch_rad = self.pitch.to_radians();

        let direction = Vector3::new(
            yaw_rad.cos() * pitch_rad.cos(),
            yaw_rad.sin() * pitch_rad.cos(),
            pitch_rad.sin(),
        )
        .normalize();

        transform.position = self.target - (direction * self.distance);

        let forward = (self.target - transform.position).normalize();
        let world_up = Vector3::new(0.0, 0.0, -1.0);
        let right = forward.cross(&world_up).normalize();
        let up = right.cross(&forward).normalize();

        // Camera looks down -Z, so map local -Z to the forward direction explicitly.
        let rotation_matrix = Matrix3::from_columns(&[right, up, -forward]);
        transform.rotation = UnitQuaternion::from_matrix(&rotation_matrix);
    }

    fn right(&self) -> Vector3<f32> {
        let forward = self.direction();
        let world_up = Vector3::new(0.0, 0.0, -1.0);
        forward.cross(&world_up).normalize()
    }

    fn up(&self) -> Vector3<f32> {
        let forward = self.direction();
        self.right().cross(&forward).normalize()
    }

    fn direction(&self) -> Vector3<f32> {
        let yaw_rad = self.yaw.to_radians();
        let pitch_rad = self.pitch.to_radians();
        Vector3::new(
            yaw_rad.cos() * pitch_rad.cos(),
            yaw_rad.sin() * pitch_rad.cos(),
            pitch_rad.sin(),
        )
        .normalize()
    }

    fn pitch_yaw(&mut self, delta_x: f32, delta_y: f32) {
        self.yaw += delta_x * self.sensitivity;
        self.pitch += delta_y * self.sensitivity;
        self.pitch = self.pitch.clamp(-89.9, 89.9);
    }

    fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let right = self.right();
        let up = self.up();
        let pan_scale = self.distance * (self.sensitivity * self.sensitivity);

        self.target -= (right * delta_x * self.sensitivity) * pan_scale;
        self.target -= (up * delta_y * self.sensitivity) * pan_scale;
    }

    fn zoom(&mut self, delta: f32) {
        self.distance -= delta * self.sensitivity * 10.0;
        self.distance = self.distance.clamp(10.0, 300.0);
    }
}

/// First-person camera controller for mouse-look.
#[allow(dead_code)]
#[derive(Component, Debug)]
#[require(TransformComponent)]
pub struct FirstPersonCameraComponent {
    pub yaw: f32,
    pub pitch: f32,
    pub sensitivity: f32,
}

#[allow(dead_code)]
impl FirstPersonCameraComponent {
    fn look(&mut self, delta_x: f32, delta_y: f32, transform: &mut TransformComponent) {
        self.yaw += delta_x * self.sensitivity;
        self.pitch += delta_y * self.sensitivity;
        self.pitch = self.pitch.clamp(-89.9, 89.9);

        let yaw_rad = self.yaw.to_radians();
        let pitch_rad = self.pitch.to_radians();

        let forward = Vector3::new(
            yaw_rad.cos() * pitch_rad.cos(),
            yaw_rad.sin() * pitch_rad.cos(),
            pitch_rad.sin(),
        )
        .normalize();

        let world_up = Vector3::new(0.0, 0.0, -1.0);
        let right = forward.cross(&world_up).normalize();
        let up = right.cross(&forward).normalize();

        let rotation_matrix = Matrix3::from_columns(&[right, up, -forward]);
        transform.rotation = UnitQuaternion::from_matrix(&rotation_matrix);
    }
}

#[allow(dead_code)]
/// Applies orbit camera input to the active camera entity.
pub fn apply_orbit_camera_input(
    active_camera: Res<ActiveCamera>,
    input_state: Res<InputState>,
    mut query: Query<(&mut TransformComponent, &mut OrbitCameraComponent)>,
) {
    let Some(camera_entity) = active_camera.0 else {
        return;
    };

    let Ok((mut transform, mut orbit)) = query.get_mut(camera_entity) else {
        return;
    };

    let (dx, dy) = input_state.mouse_delta;
    if input_state.mouse_buttons.left {
        orbit.pitch_yaw(dx, dy);
    } else if input_state.mouse_buttons.middle {
        orbit.pan(dx, -dy);
    } else if input_state.mouse_buttons.right {
        orbit.zoom(dy);
    }

    if input_state.scroll_delta != 0.0 {
        orbit.zoom(input_state.scroll_delta);
    }

    orbit.apply_to_transform(&mut transform);
}

/// Applies first-person mouse look to the active camera entity.
#[allow(dead_code)]
pub fn apply_first_person_camera_input(
    active_camera: Res<ActiveCamera>,
    input_state: Res<InputState>,
    mut query: Query<(&mut TransformComponent, &mut FirstPersonCameraComponent)>,
) {
    let Some(camera_entity) = active_camera.0 else {
        return;
    };

    let Ok((mut transform, mut controller)) = query.get_mut(camera_entity) else {
        return;
    };

    let (dx, dy) = input_state.mouse_delta;
    if input_state.mouse_buttons.left {
        controller.look(dx, dy, &mut transform);
    }
}
