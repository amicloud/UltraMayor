// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

mod action;
mod action_manager;
mod basic_physics_system;
mod camera;
mod camera_input;
mod camera_resource;
mod frustum;
mod handles;
mod input;
mod material;
mod material_component;
mod material_resource;
mod mesh;
mod mesh_component;
mod mesh_resource;
mod render_data_manager;
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
use nalgebra::Vector3;
use rand::random_range;
use renderer::Renderer;
use std::cell::RefCell;
use std::ffi::OsStr;
use std::rc::Rc;
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;

use crate::basic_physics_system::BasicPhysicsSystem;
use crate::camera::Camera;
use crate::camera_input::{
    apply_camera_input, update_camera_messages, ActiveCamera, CameraInputMessage, CameraInputState,
};
use crate::input::MouseButton;
use crate::material_component::MaterialComponent;
use crate::mesh::Mesh;
use crate::mesh_component::MeshComponent;
use crate::mesh_resource::MeshResource;
use crate::render_data_manager::RenderResourceManager;
use crate::render_instance::RenderInstance;
use crate::render_queue::RenderQueue;
use crate::render_system::RenderSystem;
use crate::renderer::RenderParams;
use crate::transform_component::TransformComponent;
use crate::velocity_component::VelocityComponent;
#[derive(Default)]
#[allow(dead_code)]
struct MouseState {
    x: f32,
    y: f32,
    p_x: f32,
    p_y: f32,
    left_pressed: bool,
    middle_pressed: bool,
    right_pressed: bool,
    other_pressed: bool,
    back_pressed: bool,
    forward_pressed: bool,
}

pub struct Engine {}

impl Engine {
    pub fn run_slintless(&self) {
        unsafe {
            let (gl, window, mut events_loop, _context) = Self::create_sdl2_context();

            let mut world = World::new();
            world.insert_resource(MeshResource::default());
            world.insert_resource(RenderQueue::default());
            world.insert_resource(RenderResourceManager::new());
            world.insert_resource(CameraInputState::default());
            world.insert_resource(ActiveCamera::default());
            world.init_resource::<Messages<CameraInputMessage>>();

            let schedule = Rc::new(RefCell::new({
                let mut s = Schedule::default();
                s.add_systems(
                    (
                        BasicPhysicsSystem::update,
                        apply_camera_input,
                        RenderSystem::extract_render_data,
                        update_camera_messages,
                    )
                        .chain(),
                );
                s
            }));

            let gl = Rc::new(gl); // Wrap in Rc for shared ownership
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

            let camera_handle = world
                .get_resource_mut::<RenderResourceManager>()
                .unwrap()
                .camera_manager
                .add_camera(Camera::new(16.0 / 9.0));
            if let Some(mut active_camera) = world.get_resource_mut::<ActiveCamera>() {
                active_camera.0 = camera_handle;
            } else {
                world.insert_resource(ActiveCamera(camera_handle));
            }

            let test_gltfs = [
                "resources/models/cube/Cube.gltf",
                "resources/models/normal_tangent_test/NormalTangentMirrorTest.gltf",
                "resources/models/suzanne/Suzanne.gltf",
            ];

            let test_objects = {
                let mut render_data_manager =
                    world.get_resource_mut::<RenderResourceManager>().unwrap();

                render_data_manager
                    .texture_manager
                    .create_default_normal_map(&gl);

                let mut objects = Vec::with_capacity(test_gltfs.len());
                for model_path in test_gltfs {
                    let test_gltf = OsStr::new(model_path);
                    let mesh_handle = render_data_manager
                        .mesh_manager
                        .add_mesh(Mesh::from_gltf(test_gltf).unwrap(), &gl);

                    let mut material_handles = render_data_manager
                        .load_materials_from_gltf(
                            &gl,
                            test_gltf,
                            OsStr::new("resources/shaders/pbr.vert"),
                            OsStr::new("resources/shaders/pbr.frag"),
                        )
                        .expect("Failed to load glTF materials");

                    let m_handle = material_handles.pop().expect("No materials found in glTF");

                    objects.push((mesh_handle, m_handle));
                }
                objects
            };

            let t_range = 2.0;
            for _ in 0..5 {
                for (mesh_handle, material_handle) in &test_objects {
                    // Random position
                    let pos = Vector3::new(
                        random_range(-10.0..10.0),
                        random_range(-10.0..10.0),
                        random_range(-10.0..10.0),
                    );

                    // Random translational velocity
                    let translational = Vector3::new(
                        random_range(-t_range..t_range),
                        random_range(-t_range..t_range),
                        random_range(-t_range..t_range),
                    );

                    // Random angular velocity
                    let angular = Vector3::new(
                        random_range(-1.0..1.0),
                        random_range(-1.0..1.0),
                        random_range(-1.0..1.0),
                    );

                    // Static position
                    // let pos = Vector3::zeros();

                    // // Random translational velocity
                    // let translational = Vector3::zeros();

                    // // Random angular velocity
                    // let angular = Vector3::zeros();

                    let scale = 10.0;
                    // Spawn test objects
                    world.spawn((
                        TransformComponent {
                            position: pos,
                            rotation: nalgebra::UnitQuaternion::identity(),
                            scale: Vector3::new(scale, scale, scale),
                        },
                        VelocityComponent {
                            translational,
                            angular,
                        },
                        MeshComponent {
                            mesh_id: *mesh_handle,
                        },
                        MaterialComponent {
                            material_id: *material_handle,
                        },
                    ));
                }
            }
            let mut last_frame = Instant::now();
            let mut accumulator = Duration::ZERO;
            let fixed_dt = Duration::from_millis(16); // ~60 Hz
            let target_frame = Duration::from_millis(16); // ~60 FPS max

            'render: loop {
                let frame_start = Instant::now();
                {
                    let mut camera_messages = world
                        .get_resource_mut::<Messages<CameraInputMessage>>()
                        .expect("CameraInputMessage resource not found");

                    for event in events_loop.poll_iter() {
                        match event {
                            sdl2::event::Event::Quit { .. } => {
                                break 'render;
                            }
                            sdl2::event::Event::MouseMotion { x, y, .. } => {
                                camera_messages.write(CameraInputMessage::MouseMove {
                                    x: x as f32,
                                    y: y as f32,
                                });
                            }
                            sdl2::event::Event::MouseWheel { y, direction, .. } => {
                                let mut delta = y as f32;
                                if direction == sdl2::mouse::MouseWheelDirection::Flipped {
                                    delta = -delta;
                                }
                                camera_messages.write(CameraInputMessage::MouseScroll { delta: delta * 10.0 });
                            }
                            sdl2::event::Event::MouseButtonDown { mouse_btn, .. } => {
                                let button = MouseButton::from(mouse_btn);
                                camera_messages.write(CameraInputMessage::MouseDown { button });
                            }
                            sdl2::event::Event::MouseButtonUp { mouse_btn, .. } => {
                                let button = MouseButton::from(mouse_btn);
                                camera_messages.write(CameraInputMessage::MouseUp { button });
                            }
                            _ => {}
                        }
                    }

                    let width = window.size().0 as f32;
                    let height = window.size().1 as f32;
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
                        schedule.borrow_mut().run(&mut world);
                        accumulator -= fixed_dt;
                    }

                    // 1. Extract instance data AFTER systems run
                    let instances: Vec<RenderInstance> = {
                        let render_queue = world
                            .get_resource::<RenderQueue>()
                            .expect("RenderQueue resource not found");
                        render_queue.instances.clone()
                    };

                    // 2. Get the render data manager
                    let mut render_data_manager = world
                        .get_resource_mut::<RenderResourceManager>()
                        .expect("RenderDataManager resource not found");

                    // 3. Render
                    renderer.render(render_params, &mut *render_data_manager, instances);
                }
                window.gl_swap_window();
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
}