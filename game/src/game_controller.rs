use bevy_ecs::prelude::*;
use engine::input::InputStateResource;
use engine::physics_component::{PhysicsComponent, PhysicsType};
use engine::{
    ActiveCamera, CameraComponent, CollisionLayer, ConvexCollider, RenderBodyComponent,
    RenderBodyHandle, TimeResource, TransformComponent, VelocityComponent, WorldBasis,
};
use glam::Vec3;
use sdl2::keyboard::Keycode;

#[derive(Resource)]
pub struct ProjectileSpawner {
    pub sphere_handle: RenderBodyHandle,
    pub cooldown: f32,
    pub cooldown_timer: f32,
    pub speed: f32,
    pub scale: f32,
}

pub fn do_gameplay(
    mut commands: Commands,
    active_camera: Res<ActiveCamera>,
    input_state: Res<InputStateResource>,
    camera_query: Query<&TransformComponent, With<CameraComponent>>,
    mut spawner: ResMut<ProjectileSpawner>,
    time: Res<TimeResource>,
    mut world: ResMut<WorldBasis>,
) {
    let g = world.gravity_vector();
    let right = world.right();
    let up = world.up();
    let ratio = 100.0;
    if input_state.key_held(Keycode::I) {
        world.set_gravity_vector(g + (up / ratio));
    }
    if input_state.key_held(Keycode::J) {
        world.set_gravity_vector(g + (-right / ratio));
    }
    if input_state.key_held(Keycode::K) {
        world.set_gravity_vector(g + (-up / ratio));
    }
    if input_state.key_held(Keycode::L) {
        world.set_gravity_vector(g + (right / ratio));
    }

    // if spawner.cooldown_timer > 0.0 {
    //     spawner.cooldown_timer = (spawner.cooldown_timer - time.simulation_fixed_dt().as_secs_f32()).max(0.0);
    // }

    // if spawner.cooldown_timer > 0.0 || !input_state.mouse_button_pressed(engine::MouseButton::Left)
    // {
    //     return;
    // }

    // let Some(camera_entity) = active_camera.0 else {
    //     return;
    // };

    // let Ok(camera_transform) = camera_query.get(camera_entity) else {
    //     return;
    // };

    // let forward = camera_transform.rotation * Vec3::NEG_Z;
    // let spawn_position = camera_transform.position + (forward * 3.0) * (spawner.scale * 1.5);

    // commands.spawn((
    //     TransformComponent {
    //         position: spawn_position,
    //         rotation: camera_transform.rotation,
    //         scale: Vec3::splat(spawner.scale),
    //     },
    //     VelocityComponent {
    //         translational: forward * spawner.speed,
    //         angular: Vec3::ZERO,
    //     },
    //     RenderBodyComponent {
    //         render_body_id: spawner.sphere_handle,
    //     },
    //     ConvexCollider::sphere(spawner.scale, CollisionLayer::Default),
    //     PhysicsComponent {
    //         mass: 1.0,
    //         physics_type: PhysicsType::Dynamic,
    //         friction: 0.2,
    //         drag_coefficient: 0.02,
    //         angular_drag_coefficient: 0.02,
    //         restitution: 0.6,
    //         local_inertia: glam::Mat3::IDENTITY,
    //     },
    // ));

    // spawner.cooldown_timer = spawner.cooldown;
}
