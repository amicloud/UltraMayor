// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

mod action;
mod action_manager;
mod camera_component;
mod collider_component;
mod collision_system;
mod frustum;
mod handles;
pub mod input;
mod material;
mod material_component;
mod material_resource;
mod mesh;
mod mesh_resource;
mod model_loader;
mod movement_system;
pub mod physics_component;
pub mod physics_resource;
mod physics_system;
mod render_body;
mod render_body_component;
mod render_body_resource;
mod render_instance;
mod render_queue;
mod render_resource_manager;
mod render_system;
mod renderer;
mod shader;
mod shader_resource;
mod texture;
mod texture_resource_manager;
mod transform_component;
mod velocity_component;
pub mod world_basis;
use bevy_ecs::prelude::*;
use glam::Mat4;
use glam::Vec3;
use glow::HasContext;
use renderer::Renderer;
use std::rc::Rc;
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;

use crate::collision_system::CollisionSystem;
use crate::input::InputStateResource;
use crate::mesh::AABB;
use crate::mesh_resource::MeshResource;
use crate::movement_system::MovementSystem;
use crate::physics_resource::Impulse;
use crate::physics_resource::PhysicsResource;
use crate::physics_system::PhysicsSystem;
use crate::render_instance::RenderInstance;
use crate::render_queue::RenderQueue;
use crate::render_resource_manager::RenderResourceManager;
use crate::render_system::RenderSystem;
use crate::renderer::{CameraRenderData, RenderParams};

pub use crate::camera_component::{ActiveCamera, CameraComponent};
pub use crate::collider_component::{ColliderComponent, CollisionLayer};
pub use crate::handles::{MaterialHandle, MeshHandle, RenderBodyHandle};
pub use crate::input::MouseButton;
pub use crate::material_component::MaterialComponent;
pub use crate::render_body_component::RenderBodyComponent;
pub use crate::transform_component::TransformComponent;
pub use crate::velocity_component::VelocityComponent;
pub use crate::world_basis::WorldBasis;
pub struct Engine {
    pub world: World,
    pub schedule: Schedule,
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

        let mut schedule = Schedule::default();
        // Engine-only systems. Game code adds its own systems to this schedule.
        schedule.add_systems(
            (
                MovementSystem::update,
                CollisionSystem::update_world_aabb_cache,
                CollisionSystem::generate_contacts,
                CollisionSystem::resolve_contacts,
                PhysicsSystem::apply_impulses,
                PhysicsSystem::update_bodies,
                RenderSystem::extract_render_data,
            )
                .chain(),
        );

        let mut render_data_manager = world
            .get_resource_mut::<RenderResourceManager>()
            .expect("RenderResourceManager resource not found");

        render_data_manager
            .texture_manager
            .create_default_normal_map(&gl);

        Engine {
            world,
            schedule,
            gl,
            window,
            events_loop,
            _gl_context: gl_context,
        }
    }

    pub fn run(&mut self) {
        unsafe {
            let gl = self.gl.clone();
            let version = gl.get_parameter_string(glow::VERSION);
            println!("OpenGL Version: {}", version);

            let shading_language_version = gl.get_parameter_string(glow::SHADING_LANGUAGE_VERSION);
            println!("GLSL Version: {}", shading_language_version);

            let major_version = gl.get_parameter_i32(glow::MAJOR_VERSION);
            let minor_version = gl.get_parameter_i32(glow::MINOR_VERSION);
            println!(
                "OpenGL Major Version: {}. OpenGL Minor Version: {}",
                major_version, minor_version
            );

            // Initialize renderer
            let mut renderer = Renderer::new(gl.clone());
            let mut last_frame = Instant::now();
            let mut accumulator = Duration::ZERO;
            let target_simulation_dt = Duration::from_millis(16); // ~60 Hz
            let target_frame_time = Duration::from_millis(16); // ~60 FPS max

            'render: loop {
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

                    accumulator += frame_time;

                    while accumulator >= target_simulation_dt {
                        self.schedule.run(&mut self.world);
                        accumulator -= target_simulation_dt;
                    }

                    // 1. Extract instance data AFTER systems run
                    let instances: Vec<RenderInstance> = {
                        let render_queue = self
                            .world
                            .get_resource::<RenderQueue>()
                            .expect("RenderQueue resource not found");
                        render_queue.instances.clone()
                    };

                    // 2. Compute camera matrices from ECS state (camera is fully game-driven).
                    let camera_data = Self::build_camera_render_data(
                        &mut self.world,
                        render_params.width,
                        render_params.height,
                    );

                    // 3. Get the render data manager
                    let mut render_data_manager = self
                        .world
                        .get_resource_mut::<RenderResourceManager>()
                        .expect("RenderDataManager resource not found");

                    // 4. Render
                    renderer.render(
                        render_params,
                        &mut *render_data_manager,
                        instances,
                        camera_data,
                    );
                }
                self.window.gl_swap_window();
                let frame_time = frame_start.elapsed();
                if frame_time < target_frame_time {
                    sleep(target_frame_time - frame_time);
                }
            }
        }
    }

    unsafe fn create_sdl2_context() -> (
        glow::Context,
        sdl2::video::Window,
        sdl2::EventPump,
        sdl2::video::GLContext,
    ) { unsafe {
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
        let gl = glow::Context::from_loader_function(|s| video.gl_get_proc_address(s) as *const _);
        let event_loop = sdl.event_pump().unwrap();

        (gl, window, event_loop, gl_context)
    }}

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
            .unwrap_or_else(|| Mat4::IDENTITY);

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

    pub fn add_impulse(&mut self, impulse: Impulse) {
        self.world
            .get_resource_mut::<PhysicsResource>()
            .unwrap()
            .add_impulse(impulse.entity, impulse.linear, impulse.angular);
    }
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

impl Engine {
    pub fn aabb_from_render_body(&self, render_body_id: RenderBodyHandle) -> Option<AABB> {
        let render_resource_manager = self.world.get_resource::<RenderResourceManager>()?;
        let render_body = render_resource_manager
            .render_body_manager
            .get_render_body(render_body_id)?;

        let mut combined: Option<AABB> = None;
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

    pub fn collider_from_render_body(
        &self,
        render_body_id: RenderBodyHandle,
        layer: CollisionLayer,
    ) -> Option<ColliderComponent> {
        self.aabb_from_render_body(render_body_id)
            .map(|aabb| ColliderComponent::new(aabb, layer))
    }
}

fn transform_aabb_with_mat4(aabb: AABB, transform: &Mat4) -> AABB {
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

    AABB {
        min: world_min,
        max: world_max,
    }
}

fn union_aabb(a: AABB, b: AABB) -> AABB {
    AABB {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}
