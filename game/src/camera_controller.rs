use bevy_ecs::prelude::*;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use sdl2::keyboard::Keycode;
use engine::{
    ActiveCamera, CameraComponent, MouseButton, TransformComponent,
    VelocityComponent, WorldBasis,
};
use engine::input::InputStateResource;

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
    pub fn apply_to_transform(&mut self, transform: &mut TransformComponent, world: &WorldBasis) {
        let yaw_rad = self.yaw.to_radians();
        let pitch_rad = self.pitch.to_radians();

        let direction = Vector3::new(
            yaw_rad.cos() * pitch_rad.cos(),
            yaw_rad.sin() * pitch_rad.cos(),
            pitch_rad.sin(),
        )
        .normalize();
        self.target.z = 0.0; // Keep the target on the ground plane.

        transform.position = self.target - (direction * self.distance);

        let forward = (self.target - transform.position).normalize();
        let world_up = -world.up();
        let right = forward.cross(&world_up).normalize();
        let up = right.cross(&forward).normalize();

        // Camera looks down -Z, so map local -Z to the forward direction explicitly.
        let rotation_matrix = Matrix3::from_columns(&[right, up, -forward]);
        transform.rotation = UnitQuaternion::from_matrix(&rotation_matrix);
    }

    fn right(&self, world: &WorldBasis) -> Vector3<f32> {
        let forward = self.direction();
        forward.cross(&world.up()).normalize()
    }

    fn up(&self, world: &WorldBasis) -> Vector3<f32> {
        let forward = self.direction();
        self.right(world).cross(&forward).normalize()
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

    fn pan(&mut self, delta_x: f32, delta_y: f32, world: &WorldBasis) {
        let right = self.right(world);
        let up = self.up(world);
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
pub struct FlyingCameraComponent {
    pub yaw: f32,
    pub pitch: f32,
    pub sensitivity: f32,
}

/// Movement settings for the flying camera.
#[derive(Component, Debug)]
#[require(TransformComponent, VelocityComponent)]
pub struct FlyingCameraMovementComponent {
    pub speed: f32,
}

#[allow(dead_code)]
impl FlyingCameraComponent {
    fn look(
        &mut self,
        delta_x: f32,
        delta_y: f32,
        transform: &mut TransformComponent,
        world: &WorldBasis,
    ) {
        self.yaw -= delta_x * self.sensitivity;
        self.pitch -= delta_y * self.sensitivity;
        self.pitch = self.pitch.clamp(-89.9, 89.9);

        let yaw_rad = self.yaw.to_radians();
        let pitch_rad = self.pitch.to_radians();

        let forward = Vector3::new(
            yaw_rad.cos() * pitch_rad.cos(),
            yaw_rad.sin() * pitch_rad.cos(),
            pitch_rad.sin(),
        )
        .normalize();

        let right = forward.cross(&world.up()).normalize();
        let up = right.cross(&forward).normalize();

        let rotation_matrix = Matrix3::from_columns(&[right, up, -forward]);
        transform.rotation = UnitQuaternion::from_matrix(&rotation_matrix);
    }
}

#[allow(dead_code)]
/// Applies orbit camera input to the active camera entity.
pub fn apply_orbit_camera_input(
    active_camera: Res<ActiveCamera>,
    input_state: Res<InputStateResource>,
    world_basis: Res<WorldBasis>,
    mut query: Query<(&mut TransformComponent, &mut OrbitCameraComponent)>,
) {
    let Some(camera_entity) = active_camera.0 else {
        return;
    };

    let Ok((mut transform, mut orbit)) = query.get_mut(camera_entity) else {
        return;
    };

    let (dx, dy) = input_state.mouse_delta;
    if input_state.mouse_button_held(MouseButton::Left) {
        orbit.pitch_yaw(dx, dy);
    } else if input_state.mouse_button_held(MouseButton::Middle) {
        orbit.pan(dx, -dy, &world_basis);
    } else if input_state.mouse_button_held(MouseButton::Right) {
        orbit.zoom(dy);
    }

    if input_state.scroll_delta != 0.0 {
        orbit.zoom(input_state.scroll_delta);
    }

    orbit.apply_to_transform(&mut transform, &world_basis);
}

/// Applies first-person mouse look to the active camera entity.
pub fn apply_flying_camera_input(
    active_camera: Res<ActiveCamera>,
    input_state: Res<InputStateResource>,
    world_basis: Res<WorldBasis>,
    mut query: Query<(&mut TransformComponent, &mut FlyingCameraComponent)>,
) {
    let Some(camera_entity) = active_camera.0 else {
        return;
    };

    let Ok((mut transform, mut camera)) = query.get_mut(camera_entity) else {
        return;
    };

    let (dx, dy) = input_state.mouse_delta;
    if input_state.mouse_button_held(MouseButton::Left) {
        camera.look(dx, dy, &mut transform, &world_basis);
    }

    let arrow_sensitivity = 10.0;

    if input_state.key_held(Keycode::Up) {
        camera.look(0.0, -1.0 * arrow_sensitivity, &mut transform, &world_basis);
    }
    if input_state.key_held(Keycode::Down) {
        camera.look(0.0, 1.0 * arrow_sensitivity, &mut transform, &world_basis);
    }
    if input_state.key_held(Keycode::Left) {
        camera.look(-1.0 * arrow_sensitivity, 0.0, &mut transform, &world_basis);
    }
    if input_state.key_held(Keycode::Right) {
        camera.look(1.0 * arrow_sensitivity, 0.0, &mut transform, &world_basis);
    }
}

/// Applies WASD + Space/Shift to the active flying camera's velocity.
pub fn apply_flying_camera_movement(
    active_camera: Res<ActiveCamera>,
    input_state: Res<InputStateResource>,
    mut query: Query<(
        &TransformComponent,
        &FlyingCameraMovementComponent,
        &mut VelocityComponent,
    )>,
) {
    let Some(camera_entity) = active_camera.0 else {
        return;
    };

    let Ok((transform, controller, mut velocity)) = query.get_mut(camera_entity) else {
        return;
    };

    let forward = transform
        .rotation
        .transform_vector(&Vector3::new(0.0, 0.0, -1.0));
    let right = transform
        .rotation
        .transform_vector(&Vector3::new(1.0, 0.0, 0.0));
    let up = transform
        .rotation
        .transform_vector(&Vector3::new(0.0, 1.0, 0.0));

    let mut direction = Vector3::new(0.0, 0.0, 0.0);
    if input_state.key_held(Keycode::W) {
        direction += forward;
    }
    if input_state.key_held(Keycode::S) {
        direction -= forward;
    }
    if input_state.key_held(Keycode::D) {
        direction += right;
    }
    if input_state.key_held(Keycode::A) {
        direction -= right;
    }
    if input_state.key_held(Keycode::LShift) {
        direction += up;
    }
    if input_state.key_held(Keycode::LCTRL) {
        direction -= up;
    }

    if input_state.key_pressed(Keycode::P) {
        println!(
            "Position: {:?}, Velocity: {:?}",
            transform.position, velocity.translational
        );
    }

    if direction.magnitude() > 0.0 {
        velocity.translational = direction.normalize() * controller.speed;
    }

    velocity.translational.x -= velocity.translational.x * 0.1;
    velocity.translational.y -= velocity.translational.y * 0.1;
    velocity.translational.z -= velocity.translational.z * 0.1;
}

pub fn apply_switch_camera_input(
    input_state: Res<InputStateResource>,
    mut active_camera: ResMut<ActiveCamera>,
    query: Query<(Entity, &CameraComponent)>,
) {
    if input_state.key_pressed(Keycode::V) {
        println!("Toggling active camera.");
        for (entity, _camera) in &query {
            if Some(entity) != active_camera.0 {
                active_camera.0 = Some(entity);
                println!("Switched active camera to entity {:?}", entity);
                break;
            }
        }
    }
}
