use bevy_ecs::prelude::*;
use engine::input::InputStateResource;
use engine::{
    ActiveCamera, CameraComponent, MouseButton, TransformComponent, VelocityComponent, WorldBasis,
};
use glam::{Mat3, Quat, Vec3};
use sdl2::keyboard::Keycode;

/// Orbit-style camera parameters controlled by input.
#[derive(Component, Debug)]
#[require(TransformComponent)]
pub struct OrbitCameraComponent {
    pub target: Vec3,
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

        let direction = Vec3::new(
            yaw_rad.cos() * pitch_rad.cos(),
            yaw_rad.sin() * pitch_rad.cos(),
            pitch_rad.sin(),
        )
        .normalize();

        // self.target.z = 0.0; // Keep the target on the ground plane.

        transform.position = self.target - (direction * self.distance);

        let forward = (self.target - transform.position).normalize();
        let world_up = world.up();
        let right = forward.cross(world_up).normalize();
        let up = right.cross(forward).normalize();

        let rotation_matrix = Mat3::from_cols(right, up, -forward);
        transform.rotation = Quat::from_mat3(&rotation_matrix);
    }

    fn right(&self, world: &WorldBasis) -> Vec3 {
        let forward = self.direction();
        forward.cross(world.up()).normalize()
    }

    fn up(&self, world: &WorldBasis) -> Vec3 {
        let forward = self.direction();
        self.right(world).cross(forward).normalize()
    }

    fn direction(&self) -> Vec3 {
        let yaw_rad = self.yaw.to_radians();
        let pitch_rad = self.pitch.to_radians();
        Vec3::new(
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
#[require(TransformComponent, VelocityComponent)]
pub struct FlyingCameraComponent {
    pub yaw: f32,
    pub pitch: f32,
    pub sensitivity: f32,
    pub speed: f32,
}

/// Player marker + movement tuning for impulse-based motion.
#[derive(Component, Debug)]
#[require(TransformComponent, VelocityComponent)]
pub struct PlayerComponent {
    pub speed: f32,
}

#[allow(dead_code)]
impl FlyingCameraComponent {
    fn apply_to_transform(&self, transform: &mut TransformComponent, world: &WorldBasis) {
        let yaw_rad = self.yaw.to_radians();
        let pitch_rad = self.pitch.to_radians();

        let forward = Vec3::new(
            yaw_rad.cos() * pitch_rad.cos(),
            yaw_rad.sin() * pitch_rad.cos(),
            pitch_rad.sin(),
        )
        .normalize();

        let right = forward.cross(world.up()).normalize();
        let up = right.cross(forward).normalize();

        let rotation_matrix = Mat3::from_cols(right, up, -forward);
        transform.rotation = Quat::from_mat3(&rotation_matrix);
    }

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

        self.apply_to_transform(transform, world);
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

/// Initializes newly spawned flying cameras so their transform matches their yaw/pitch.
pub fn initialize_flying_camera_rotation(
    world_basis: Res<WorldBasis>,
    mut query: Query<
        (&mut TransformComponent, &FlyingCameraComponent),
        Added<FlyingCameraComponent>,
    >,
) {
    for (mut transform, camera) in &mut query {
        camera.apply_to_transform(&mut transform, &world_basis);
    }
}

/// Applies WASD + Space/Shift to the active flying camera's velocity.
pub fn apply_flying_camera_movement(
    active_camera: Res<ActiveCamera>,
    input_state: Res<InputStateResource>,
    mut query: Query<(
        &TransformComponent,
        &FlyingCameraComponent,
        &mut VelocityComponent,
    )>,
) {
    let Some(camera_entity) = active_camera.0 else {
        return;
    };

    let Ok((transform, controller, mut velocity)) = query.get_mut(camera_entity) else {
        return;
    };

    let forward = transform.rotation * Vec3::NEG_Z;
    let right = transform.rotation * Vec3::X;
    let up = transform.rotation * Vec3::Y;

    let mut direction = Vec3::new(0.0, 0.0, 0.0);
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

    if direction.length() > 0.0 {
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

/// Applies WASD impulses to the player based on the active camera's facing.
pub fn apply_player_movement_impulses(
    input_state: Res<InputStateResource>,
    active_camera: Res<ActiveCamera>,
    world_basis: Res<WorldBasis>,
    camera_query: Query<&TransformComponent, With<CameraComponent>>,
    mut player_query: Query<(&PlayerComponent, &mut VelocityComponent), Without<CameraComponent>>,
) {
    let (camera_forward, camera_right) = if let Some(camera_entity) = active_camera.0 {
        if let Ok(camera_transform) = camera_query.get(camera_entity) {
            let forward = camera_transform.rotation * Vec3::NEG_Y;
            let right = camera_transform.rotation * Vec3::X;
            (forward, right)
        } else {
            (world_basis.forward(), world_basis.right())
        }
    } else {
        (world_basis.forward(), world_basis.right())
    };

    let mut forward = Vec3::new(camera_forward.x, camera_forward.y, 0.0);
    let mut right = Vec3::new(camera_right.x, camera_right.y, 0.0);
    if forward.length_squared() > 0.0 {
        forward = forward.normalize();
    }
    if right.length_squared() > 0.0 {
        right = right.normalize();
    }

    let mut move_dir = Vec3::ZERO;
    if input_state.key_held(Keycode::W) {
        move_dir -= right;
    }
    if input_state.key_held(Keycode::S) {
        move_dir += right;
    }
    if input_state.key_held(Keycode::D) {
        move_dir -= forward;
    }
    if input_state.key_held(Keycode::A) {
        move_dir += forward;
    }
    if input_state.key_held(Keycode::Space) {
        move_dir += world_basis.up();
    }

    if move_dir.length_squared() == 0.0 {
        return;
    }

    let move_dir = move_dir.normalize();
    for (player, mut velocity) in &mut player_query {
        velocity.angular += move_dir * player.speed;
    }
}

/// Keeps orbit cameras locked to the player's position.
pub fn update_orbit_camera_target(
    world_basis: Res<WorldBasis>,
    player_query: Query<&TransformComponent, With<PlayerComponent>>,
    mut orbit_query: Query<
        (&mut TransformComponent, &mut OrbitCameraComponent),
        Without<PlayerComponent>,
    >,
) {
    let Ok(player_transform) = player_query.single() else {
        return;
    };

    for (mut transform, mut orbit) in &mut orbit_query {
        orbit.target = player_transform.position;
        orbit.apply_to_transform(&mut transform, &world_basis);
    }
}
