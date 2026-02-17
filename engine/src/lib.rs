// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

mod action;
mod action_manager;
pub mod components;
mod handles;
pub mod input;
mod mesh;
mod model_loader;
pub mod render;
mod sleep_component;
mod time_resource;
pub mod world_basis;
pub mod physics;
use bevy_ecs::prelude::*;
use glam::Mat4;
use glam::Vec3;
use glow::HasContext;
use render::renderer::Renderer;
use std::rc::Rc;
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;

pub use physics::collision_system::CollisionSystem;
pub use physics::gravity_resource::Gravity;
use crate::input::InputStateResource;
use crate::mesh::Aabb;
use crate::components::physics_component::PhysicsComponent;
use crate::render::mesh_resource::MeshResource;
use physics::movement_system::MovementSystem;
use physics::physics_resource::CollisionFrameData;
use physics::physics_resource::PhysicsFrameData;
use physics::physics_resource::PhysicsResource;
use physics::physics_system::PhysicsSystem;
use crate::render::render_queue::RenderQueue;
pub use crate::render::render_resource_manager::RenderResourceManager;
use crate::render::render_system::RenderSystem;
use crate::render::renderer::{CameraRenderData, RenderParams};

pub use crate::components::camera_component::{ActiveCamera, CameraComponent};
pub use crate::components::collider_component::{
    CollisionLayer, ConvexCollider, ConvexShape, MeshCollider,
};
pub use crate::handles::{MaterialHandle, MeshHandle, RenderBodyHandle};
pub use crate::input::MouseButton;
pub use crate::components::material_component::MaterialComponent;
pub use crate::components::render_body_component::RenderBodyComponent;
pub use crate::sleep_component::SleepComponent;
pub use crate::time_resource::TimeResource;
pub use crate::components::transform_component::TransformComponent;
pub use crate::components::velocity_component::VelocityComponent;
pub use crate::world_basis::WorldBasis;
pub struct Engine {
    pub world: World,
    pub game_frame_schedule: Schedule,
    pub game_simulation_schedule: Schedule,
    frame_schedule: Schedule,
    physics_schedule: Schedule,
    gl: Rc<glow::Context>,
    window: sdl2::video::Window,
    events_loop: sdl2::EventPump,
    _gl_context: sdl2::video::GLContext,
}

impl Engine {
    pub fn new() -> Self {
        let (gl, window, events_loop, gl_context) = unsafe { Self::create_sdl2_context() };
        let gl = Rc::new(gl);

        let mut world = World::new();
        world.insert_resource(MeshResource::default());
        world.insert_resource(RenderQueue::default());
        world.insert_resource(RenderResourceManager::new());
        world.insert_resource(ActiveCamera::default());
        world.insert_resource(InputStateResource::default());
        world.insert_resource(WorldBasis::canonical());
        world.insert_resource(PhysicsResource::default());
        world.insert_resource(CollisionFrameData::default());
        world.insert_resource(PhysicsFrameData::default());
        world.insert_resource(TimeResource::new(60, 120));
        world.insert_resource(Gravity::default());

        let mut physics_schedule = Schedule::default();

        // Engine-only systems. Game code adds its own systems to the game schedule.
        physics_schedule.add_systems(
            (
                MovementSystem::update,
                CollisionSystem::update_world_aabb_cache,
                CollisionSystem::update_world_dynamic_tree,
                CollisionSystem::generate_contacts,
                PhysicsSystem::physics_solver,
                PhysicsSystem::integrate_motion,
            )
                .chain(),
        );

        let mut frame_schedule = Schedule::default();
        frame_schedule.add_systems(
            (
                RenderSystem::build_render_queue,
                TimeResource::update_time_resource,
            )
                .chain(),
        );

        let game_frame_schedule = Schedule::default();
        let game_simulation_schedule = Schedule::default();

        let mut render_data_manager = world
            .get_resource_mut::<RenderResourceManager>()
            .expect("RenderResourceManager resource not found");

        render_data_manager
            .texture_manager
            .create_default_normal_map(&gl);

        Engine {
            world,
            game_frame_schedule,
            game_simulation_schedule,
            frame_schedule,
            physics_schedule,
            gl,
            window,
            events_loop,
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

        // Initialize renderer
        let mut renderer = Renderer::new(self.gl.clone());
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

        let mut _frame_count: u64 = 0;
        'render: loop {
            // dbg!(frame_count);
            _frame_count += 1;
            let frame_start = Instant::now();
            {
                let mut input_state = self
                    .world
                    .get_resource_mut::<InputStateResource>()
                    .expect("InputStateResource resource not found");

                input_state.previous_keys = input_state.current_keys.clone();
                input_state.previous_mouse_buttons = input_state.current_mouse_buttons.clone();
                input_state.mouse_delta = (0.0, 0.0);
                input_state.scroll_delta = 0.0;

                for event in self.events_loop.poll_iter() {
                    match event {
                        sdl2::event::Event::Quit { .. } => {
                            break 'render;
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

                let width = self.window.size().0 as f32;
                let height = self.window.size().1 as f32;
                let render_scale = 1.0;
                let render_params = RenderParams {
                    width: (width * render_scale) as u32,
                    height: (height * render_scale) as u32,
                };

                let now = Instant::now();
                let frame_time = now - last_frame;
                last_frame = now;

                // Update things that should run every frame, regardless of physics steps
                self.frame_schedule.run(&mut self.world);
                self.game_frame_schedule.run(&mut self.world);

                // Prevent absurd frame times (debugger pauses, window drag, etc.)
                let frame_time = frame_time.min(Duration::from_millis(250));

                accumulator += frame_time;

                let mut steps = 0;
                while accumulator >= fixed_dt && steps < max_physics_steps {
                    self.physics_schedule.run(&mut self.world);
                    self.game_simulation_schedule.run(&mut self.world);
                    accumulator -= fixed_dt;
                    steps += 1;
                }

                if steps == max_physics_steps {
                    accumulator = accumulator.min(fixed_dt);
                    println!(
                        "Warning: Reached max physics steps in a frame. Frame time: {:?}, accumulator: {:?}. Consider increasing target frame duration or decreasing simulation fixed dt.",
                        frame_time, accumulator
                    );
                }

                let camera_data = Self::build_camera_render_data(
                    &mut self.world,
                    render_params.width,
                    render_params.height,
                );

                renderer.stage_instances(
                    &self
                        .world
                        .get_resource::<RenderQueue>()
                        .expect("RenderQueue resource not found")
                        .instances,
                );

                let mut render_data_manager = self
                    .world
                    .get_resource_mut::<RenderResourceManager>()
                    .expect("RenderDataManager resource not found");

                // 4. Render
                renderer.render(render_params, &mut render_data_manager, camera_data);
            }
            self.window.gl_swap_window();
            let frame_time = frame_start.elapsed();
            if frame_time < frame_target {
                sleep(frame_target - frame_time);
            }
        }
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

        let view = transform
            .to_mat4()
            .try_inverse()
            .unwrap_or(Mat4::IDENTITY);

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
        let render_resource_manager = self.world.get_resource::<RenderResourceManager>()?;
        let render_body = render_resource_manager
            .render_body_manager
            .get_render_body(render_body_id)?;

        let mut combined: Option<Aabb> = None;
        for part in &render_body.parts {
            let mesh = render_resource_manager
                .mesh_manager
                .get_mesh(part.mesh_id)?;
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
            .get_resource::<RenderResourceManager>()?
            .render_body_manager
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
