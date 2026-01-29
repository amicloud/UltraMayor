// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

mod action;
mod action_manager;
mod basic_physics_system;
mod camera_component;
mod frustum;
mod handles;
mod input;
mod material;
mod material_component;
mod material_resource;
mod mesh;
mod mesh_component;
mod mesh_resource;
mod render_resource_manager;
mod render_instance;
mod render_queue;
mod render_system;
mod renderer;
mod shader;
mod texture;
mod texture_resource_manager;
mod transform_component;
mod velocity_component;
use bevy_ecs::message::Messages;
use bevy_ecs::prelude::*;
use glow::HasContext;
use renderer::Renderer;
use std::ffi::OsStr;
use std::rc::Rc;
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;

use crate::basic_physics_system::BasicPhysicsSystem;
use crate::mesh::Mesh;
use crate::mesh_resource::MeshResource;
use crate::render_resource_manager::RenderResourceManager;
use crate::render_instance::RenderInstance;
use crate::render_queue::RenderQueue;
use crate::render_system::RenderSystem;
use crate::renderer::{CameraRenderData, RenderParams};

pub use crate::handles::{MaterialHandle, MeshHandle};
pub use crate::material_component::MaterialComponent;
pub use crate::mesh_component::MeshComponent;
pub use crate::transform_component::TransformComponent;
pub use crate::velocity_component::VelocityComponent;
pub use crate::camera_component::{ActiveCamera, CameraComponent};
pub use crate::input::{InputMessage, MouseButton};
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
        world.init_resource::<Messages<InputMessage>>();

        let mut schedule = Schedule::default();
        // Engine-only systems. Game code adds its own systems to this schedule.
        schedule.add_systems((BasicPhysicsSystem::update, RenderSystem::extract_render_data).chain());
        
        Engine {
            world,
            schedule,
            gl,
            window,
            events_loop,
            _gl_context: gl_context,
        }
    }

    pub fn load_gltf(&mut self, gltf_path: &OsStr) -> (MeshHandle, MaterialHandle) {
        let gl = &self.gl;
        let (mesh_handle, material_handle) = {
            let mut render_data_manager = self
                .world
                .get_resource_mut::<RenderResourceManager>()
                .expect("RenderResourceManager resource not found");

            render_data_manager
                .texture_manager
                .create_default_normal_map(gl);

            let mesh_handle = render_data_manager
                .mesh_manager
                .add_mesh(Mesh::from_gltf(gltf_path).unwrap(), gl);

            let mut material_handles = render_data_manager
                .load_materials_from_gltf(
                    gl,
                    gltf_path,
                    OsStr::new("resources/shaders/pbr.vert"),
                    OsStr::new("resources/shaders/pbr.frag"),
                )
                .expect("Failed to load glTF materials");

            let material_handle = material_handles
                .pop()
                .expect("No materials found in glTF");

            (mesh_handle, material_handle)
        };

        (mesh_handle, material_handle)
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
            let fixed_dt = Duration::from_millis(16); // ~60 Hz
            let target_frame = Duration::from_millis(16); // ~60 FPS max

            'render: loop {
                let frame_start = Instant::now();
                {
                    // Engine responsibility: translate SDL2 events into generic ECS messages.
                    let mut input_messages = self.world
                        .get_resource_mut::<Messages<InputMessage>>()
                        .expect("InputMessage resource not found");

                    for event in self.events_loop.poll_iter() {
                        match event {
                            sdl2::event::Event::Quit { .. } => {
                                break 'render;
                            }
                            sdl2::event::Event::MouseMotion { x, y, .. } => {
                                input_messages.write(InputMessage::MouseMove {
                                    x: x as f32,
                                    y: y as f32,
                                });
                            }
                            sdl2::event::Event::MouseWheel { y, direction, .. } => {
                                let mut delta = y as f32;
                                if direction == sdl2::mouse::MouseWheelDirection::Flipped {
                                    delta = -delta;
                                }
                                input_messages.write(InputMessage::MouseScroll { delta: delta});
                            }
                            sdl2::event::Event::MouseButtonDown { mouse_btn, .. } => {
                                let button = MouseButton::from(mouse_btn);
                                input_messages.write(InputMessage::MouseButtonDown { button });
                            }
                            sdl2::event::Event::MouseButtonUp { mouse_btn, .. } => {
                                let button = MouseButton::from(mouse_btn);
                                input_messages.write(InputMessage::MouseButtonUp { button });
                            }
                            sdl2::event::Event::KeyDown { keycode: Some(keycode), .. } => {
                                input_messages.write(InputMessage::KeyDown { keycode });
                            }
                            sdl2::event::Event::KeyUp { keycode: Some(keycode), .. } => {
                                input_messages.write(InputMessage::KeyUp { keycode });
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
                        visualize_edges: false,
                        visualize_normals: false,
                    };
                    
                    let now = Instant::now();
                    let frame_time = now - last_frame;
                    last_frame = now;

                    accumulator += frame_time;

                    while accumulator >= fixed_dt {
                        self.schedule.run(&mut self.world);
                        // Flush input messages after all game systems have consumed them.
                        if let Some(mut messages) = self.world.get_resource_mut::<Messages<InputMessage>>() {
                            messages.update();
                        }
                        accumulator -= fixed_dt;
                    }

                    // 1. Extract instance data AFTER systems run
                    let instances: Vec<RenderInstance> = {
                        let render_queue = self.world
                            .get_resource::<RenderQueue>()
                            .expect("RenderQueue resource not found");
                        render_queue.instances.clone()
                    };

                    // 2. Compute camera matrices from ECS state (camera is fully game-driven).
                    let camera_data = Self::camera_render_data(
                        &mut self.world,
                        render_params.width,
                        render_params.height,
                    );

                    // 3. Get the render data manager
                    let mut render_data_manager = self.world
                        .get_resource_mut::<RenderResourceManager>()
                        .expect("RenderDataManager resource not found");

                    // 4. Render
                    renderer.render(render_params, &mut *render_data_manager, instances, camera_data);
                }
                self.window.gl_swap_window();
                let frame_time = frame_start.elapsed();
                if frame_time < target_frame {
                    sleep(target_frame - frame_time);
                }
            }
        }
    }

    unsafe fn create_sdl2_context() -> (
        glow::Context,
        sdl2::video::Window,
        sdl2::EventPump,
        sdl2::video::GLContext,
    ) {
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
    }

    /// Builds camera render data from the ECS world.
    fn camera_render_data(
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
            .unwrap_or_else(nalgebra::Matrix4::identity);

        let fallback_aspect = width as f32 / height as f32;
        let aspect_ratio = if camera.aspect_ratio > 0.0 {
            camera.aspect_ratio
        } else {
            fallback_aspect
        };

        let projection = nalgebra::Matrix4::new_perspective(
            aspect_ratio,
            camera.fov_y_radians,
            camera.near,
            camera.far,
        );

        Some(CameraRenderData {
            view_proj: projection * view,
            position: transform.position,
        })
    }
}