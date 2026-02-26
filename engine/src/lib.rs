// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

mod action;
mod action_manager;
pub mod assets;
pub mod audio;
pub mod components;
pub mod input;
pub mod physics;
pub mod render;
mod time_resource;
mod utils;
pub mod world_basis;
use std::{
    rc::Rc,
    thread::sleep,
    time::{Duration, Instant},
};

use bevy_ecs::prelude::*;
use glam::{Mat4, Vec3};
use glow::HasContext;

use crate::{
    assets::{
        material_resource::MaterialResource, mesh_resource::MeshResource,
        shader_resource::ShaderResource, sound_resource::SoundResource,
        texture_resource::TextureResource,
    },
    audio::{
        audio_command_queue::AudioCommandQueue,
        audio_command_queue_system::AudioCommandQueueSystem, audio_mixer::AudioMixer,
        simple_phys_audio_system::SimplePhysAudioSystem, spatial_audio_system::SpatialAudioSystem,
    },
    components::physics_component::PhysicsComponent,
    input::InputStateResource,
    physics::{
        movement_system::MovementSystem,
        physics_event_dispatcher,
        physics_resource::{CollisionFrameData, PhysicsFrameData, PhysicsResource},
        physics_system::PhysicsSystem,
    },
    render::{
        render_body_resource::RenderBodyResource,
        render_queue::RenderQueue,
        render_system::RenderSystem,
        renderer::{CameraRenderData, RenderParams, Renderer},
    },
    utils::scope_timer::ScopeTimer,
};

pub use physics::collision_system::CollisionSystem;
pub use physics::gravity_resource::Gravity;

pub use crate::assets::handles::{MaterialHandle, MeshHandle, RenderBodyHandle, SoundHandle};
pub use crate::assets::mesh::Aabb;
pub use crate::components::camera_component::{ActiveCamera, CameraComponent};
pub use crate::components::collider_component::{
    CollisionLayer, ConvexCollider, ConvexShape, MeshCollider,
};
pub use crate::components::material_component::MaterialComponent;
pub use crate::components::render_body_component::RenderBodyComponent;
pub use crate::components::sleep_component::SleepComponent;
pub use crate::components::transform_component::TransformComponent;
pub use crate::components::velocity_component::VelocityComponent;
pub use crate::input::MouseButton;
pub use crate::time_resource::TimeResource;
pub use crate::world_basis::WorldBasis;

pub struct Engine {
    pub world: World,
    pub game_frame_schedule: Schedule,
    pub game_simulation_schedule: Schedule,
    cleanup_schedule: Schedule,
    frame_schedule: Schedule,
    physics_schedule: Schedule,
    gl: Rc<glow::Context>,
    window: sdl2::video::Window,
    events_loop: sdl2::EventPump,
    renderer: Renderer,
    audio_mixer: AudioMixer,
    _gl_context: sdl2::video::GLContext,
}

impl Engine {
    pub fn new() -> Self {
        env_logger::init();
        let (gl, window, events_loop, gl_context) = unsafe { Self::create_sdl2_context() };
        let gl = Rc::new(gl);

        let mut world = World::new();
        world.insert_resource(RenderQueue::default());
        world.insert_resource(MeshResource::default());
        world.insert_resource(RenderBodyResource::default());
        world.insert_resource(MaterialResource::default());
        world.insert_resource(TextureResource::default());
        world.insert_resource(ShaderResource::default());
        world.insert_resource(ActiveCamera::default());
        world.insert_resource(InputStateResource::default());
        world.insert_resource(WorldBasis::canonical());
        world.insert_resource(PhysicsResource::default());
        world.insert_resource(CollisionFrameData::default());
        world.insert_resource(PhysicsFrameData::default());
        world.insert_resource(TimeResource::new(60, 120));
        world.insert_resource(Gravity::default());
        world.insert_resource(SoundResource::default());
        world.insert_resource(AudioCommandQueue::default());

        let mut physics_schedule = Schedule::default();

        // Engine-only systems. Game code adds its own systems to the game schedule.
        physics_schedule.add_systems(
            (
                MovementSystem::update,
                CollisionSystem::update_world_aabb_cache,
                CollisionSystem::update_world_dynamic_tree,
                CollisionSystem::generate_manifolds,
                PhysicsSystem::physics_solver,
                PhysicsSystem::integrate_motion,
            )
                .chain(),
        );
        physics_schedule.add_systems(physics_event_dispatcher::dispatch_physics_events);

        let mut frame_schedule = Schedule::default();
        frame_schedule.add_systems(
            (
                RenderSystem::build_render_queue,
                TimeResource::update_time_resource,
                AudioCommandQueueSystem::build_command_queue,
                SpatialAudioSystem::update_listener_position,
                SpatialAudioSystem::update_source_positions,
                SimplePhysAudioSystem::on_hit_audio_system,
            )
                .chain(),
        );

        let game_frame_schedule = Schedule::default();
        let game_simulation_schedule = Schedule::default();
        let mut cleanup_schedule = Schedule::default();
        cleanup_schedule.add_systems(
            (
                // RenderSystem::cleanup_render_queue,
                AudioCommandQueueSystem::clear_command_queue,
            )
                .chain(),
        );

        world
            .get_resource_mut::<TextureResource>()
            .expect("TextureResource resource not found")
            .create_default_normal_map(&gl);

        let renderer = Renderer::new(gl.clone());
        let audio_mixer = AudioMixer::default();

        Engine {
            world,
            game_frame_schedule,
            game_simulation_schedule,
            frame_schedule,
            physics_schedule,
            cleanup_schedule,
            gl,
            window,
            events_loop,
            renderer,
            audio_mixer,
            _gl_context: gl_context,
        }
    }

    pub fn run(&mut self) {
        unsafe {
            let version = self.gl.get_parameter_string(glow::VERSION);
            let shading_language_version =
                self.gl.get_parameter_string(glow::SHADING_LANGUAGE_VERSION);
            let major_version = self.gl.get_parameter_i32(glow::MAJOR_VERSION);
            let minor_version = self.gl.get_parameter_i32(glow::MINOR_VERSION);
            println!("OpenGL Version: {}", version);
            println!("GLSL Version: {}", shading_language_version);
            println!(
                "OpenGL Major Version: {}. OpenGL Minor Version: {}",
                major_version, minor_version
            );
        }

        let mut last_frame = Instant::now();
        let mut accumulator = Duration::ZERO;

        let fixed_dt: Duration = self
            .world
            .get_resource::<TimeResource>()
            .expect("TimeResource resource not found")
            .simulation_fixed_dt();

        let frame_target: Duration = self
            .world
            .get_resource::<TimeResource>()
            .expect("TimeResource resource not found")
            .target_frame_duration();

        let max_physics_steps: usize = 6;
        let mut frame_count: u64 = 0;

        'game: loop {
            log::trace!("Frame count: {}", frame_count);
            frame_count += 1;
            let frame_start = Instant::now();
            {
                let mut input_state = self
                    .world
                    .get_resource_mut::<InputStateResource>()
                    .expect("InputStateResource resource not found");

                if !Self::handle_input(&mut input_state, &mut self.events_loop) {
                    break 'game;
                }

                // Update things that should run only once per frame
                self.frame_schedule.run(&mut self.world);
                self.game_frame_schedule.run(&mut self.world);

                // Render before doing any simulation steps, so that the game feels more responsive.
                let render_params = RenderParams {
                    width: self.window.size().0,
                    height: self.window.size().1,
                };
                let camera_data = Self::build_camera_render_data(
                    &mut self.world,
                    render_params.width,
                    render_params.height,
                );

                self.renderer.stage_instances(
                    &self
                        .world
                        .get_resource::<RenderQueue>()
                        .expect("RenderQueue resource not found")
                        .instances,
                );
                {
                    let _timer = ScopeTimer::new("Render");
                    let mesh_resource = self
                        .world
                        .get_resource::<MeshResource>()
                        .expect("MeshResource resource not found");
                    let material_resource = self
                        .world
                        .get_resource::<MaterialResource>()
                        .expect("MaterialResource resource not found");
                    let texture_resource = self
                        .world
                        .get_resource::<TextureResource>()
                        .expect("TextureResource resource not found");
                    let shader_resource = self
                        .world
                        .get_resource::<ShaderResource>()
                        .expect("ShaderResource resource not found");

                    self.renderer.render(
                        render_params,
                        mesh_resource,
                        material_resource,
                        texture_resource,
                        shader_resource,
                        camera_data,
                    );
                }

                let now = Instant::now();
                let frame_time = now - last_frame;
                last_frame = now;

                // Prevent absurd frame times (debugger pauses, window drag, etc.)
                let frame_time = frame_time.min(Duration::from_millis(250));

                accumulator += frame_time;

                let mut steps = 0;
                while accumulator >= fixed_dt && steps < max_physics_steps {
                    #[cfg(not(debug_assertions))]
                    let phys_start = Instant::now();
                    {
                        let _timer = ScopeTimer::new("Physics Schedule");
                        self.physics_schedule.run(&mut self.world);
                    }
                    #[cfg(not(debug_assertions))]
                    {
                        let phys_time = phys_start.elapsed();
                        if phys_time > fixed_dt {
                            log::warn!(
                                "Physics schedule took {:?}, which is {:.2}% longer than the fixed dt of {:?}.",
                                phys_time,
                                phys_time.as_secs_f32() / fixed_dt.as_secs_f32() * 100.0,
                                fixed_dt
                            );
                        }
                    }
                    self.game_simulation_schedule.run(&mut self.world);
                    accumulator -= fixed_dt;
                    steps += 1;
                }

                if steps == max_physics_steps {
                    accumulator = accumulator.min(fixed_dt);
                }

                self.audio_mixer.make_mixer_commands(
                    &self
                        .world
                        .get_resource::<AudioCommandQueue>()
                        .expect("AudioQueue resource not found")
                        .queue,
                    self.world
                        .get_resource::<SoundResource>()
                        .expect("SoundResource resource not found"),
                );
            }
            self.cleanup_schedule.run(&mut self.world);
            let frame_time = frame_start.elapsed();
            if frame_time < frame_target {
                sleep(frame_target - frame_time);
            }
            self.window.gl_swap_window();
        }
    }

    fn handle_input(
        input_state: &mut InputStateResource,
        events_loop: &mut sdl2::EventPump,
    ) -> bool {
        input_state.previous_keys = input_state.current_keys.clone();
        input_state.previous_mouse_buttons = input_state.current_mouse_buttons.clone();
        input_state.mouse_delta = (0.0, 0.0);
        input_state.scroll_delta = 0.0;

        for event in events_loop.poll_iter() {
            match event {
                sdl2::event::Event::Quit { .. } => {
                    return false;
                }
                sdl2::event::Event::MouseMotion { xrel, yrel, .. } => {
                    input_state.mouse_delta = (xrel as f32, yrel as f32);
                }
                sdl2::event::Event::MouseWheel { y, direction, .. } => {
                    let mut delta = y as f32;
                    if direction == sdl2::mouse::MouseWheelDirection::Flipped {
                        delta = -delta;
                    }
                    input_state.scroll_delta = delta;
                }
                sdl2::event::Event::MouseButtonDown { mouse_btn, .. } => {
                    let button = MouseButton::from(mouse_btn);
                    input_state.current_mouse_buttons.insert(button);
                }
                sdl2::event::Event::MouseButtonUp { mouse_btn, .. } => {
                    let button = MouseButton::from(mouse_btn);
                    input_state.current_mouse_buttons.remove(&button);
                }
                sdl2::event::Event::KeyDown {
                    keycode: Some(keycode),
                    ..
                } => {
                    input_state.current_keys.insert(keycode);
                }
                sdl2::event::Event::KeyUp {
                    keycode: Some(keycode),
                    ..
                } => {
                    input_state.current_keys.remove(&keycode);
                }
                _ => {}
            }
        }
        true
    }

    unsafe fn create_sdl2_context() -> (
        glow::Context,
        sdl2::video::Window,
        sdl2::EventPump,
        sdl2::video::GLContext,
    ) {
        unsafe {
            let sdl = sdl2::init().unwrap();
            let video = sdl.video().unwrap();
            let gl_attr = video.gl_attr();
            gl_attr.set_context_profile(sdl2::video::GLProfile::Core);
            gl_attr.set_context_version(3, 3);
            gl_attr.set_depth_size(24);
            gl_attr.set_context_flags().forward_compatible().set();
            let window = video
                .window("Engine", 1024, 769)
                .opengl()
                .resizable()
                .build()
                .unwrap();
            let gl_context = window.gl_create_context().unwrap();
            window.gl_make_current(&gl_context).unwrap();
            let gl =
                glow::Context::from_loader_function(|s| video.gl_get_proc_address(s) as *const _);
            let event_loop = sdl.event_pump().unwrap();

            (gl, window, event_loop, gl_context)
        }
    }

    /// Builds camera render data from the ECS world.
    /// Returns `None` if there is no active camera or if the camera entity is invalid.
    fn build_camera_render_data(
        world: &mut World,
        width: u32,
        height: u32,
    ) -> Option<CameraRenderData> {
        let active = world.get_resource::<ActiveCamera>()?;
        let entity = active.0?;

        let mut query = world.query::<(&TransformComponent, &CameraComponent)>();
        let Ok((transform, camera)) = query.get(world, entity) else {
            return None;
        };

        let view = transform.to_mat4().try_inverse().unwrap_or(Mat4::IDENTITY);

        let fallback_aspect = width as f32 / height as f32;
        let aspect_ratio = if camera.aspect_ratio > 0.0 {
            camera.aspect_ratio
        } else {
            fallback_aspect
        };

        let projection =
            Mat4::perspective_rh(aspect_ratio, camera.fov_y_radians, camera.near, camera.far);

        Some(CameraRenderData {
            view_proj: projection * view,
            position: transform.position,
        })
    }
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

impl Engine {
    pub fn aabb_from_render_body(&self, render_body_id: RenderBodyHandle) -> Option<Aabb> {
        let render_body_resource = self.world.get_resource::<RenderBodyResource>()?;
        let mesh_resource = self.world.get_resource::<MeshResource>()?;
        let render_body = render_body_resource.get_render_body(render_body_id)?;

        let mut combined: Option<Aabb> = None;
        for part in &render_body.parts {
            let mesh = mesh_resource.get_mesh(part.mesh_id)?;
            let part_aabb = transform_aabb_with_mat4(mesh.aabb, &part.local_transform);
            combined = Some(match combined {
                Some(existing) => union_aabb(existing, part_aabb),
                None => part_aabb,
            });
        }

        combined
    }

    pub fn mesh_collider_from_render_body(
        &self,
        render_body_id: RenderBodyHandle,
        layer: CollisionLayer,
    ) -> Option<MeshCollider> {
        self.world
            .get_resource::<RenderBodyResource>()?
            .get_render_body(render_body_id)?;

        Some(MeshCollider::new(render_body_id, layer))
    }

    pub fn do_fake_impulse(
        velocity: &mut VelocityComponent,
        physics: &PhysicsComponent,
        impulse: glam::Vec3,
    ) {
        let delta_v = impulse / physics.mass;
        velocity.translational += delta_v;
    }
}

fn transform_aabb_with_mat4(aabb: Aabb, transform: &Mat4) -> Aabb {
    let min = aabb.min;
    let max = aabb.max;

    let corners = [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(max.x, max.y, max.z),
    ];

    let mut world_min = transform.transform_point3(corners[0]);
    let mut world_max = world_min;

    for corner in corners.iter().skip(1) {
        let world = transform.transform_point3(*corner);
        world_min = world_min.min(world);
        world_max = world_max.max(world);
    }

    Aabb {
        min: world_min,
        max: world_max,
    }
}

fn union_aabb(a: Aabb, b: Aabb) -> Aabb {
    Aabb {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}
