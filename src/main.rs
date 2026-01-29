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
mod render_texture;
mod renderer;
mod settings;
mod shader;
mod texture;
mod texture_resource_manager;
mod transform_component;
mod velocity_component;
use action_manager::ActionManager;
use bevy_ecs::message::Messages;
use bevy_ecs::prelude::*;
use glow::Context as GlowContext;
use glow::HasContext;
use log::debug;
use nalgebra::Vector3;
use rand::random_range;
use renderer::Renderer;
use settings::Settings;
use slint::Timer;
use std::cell::RefCell;
use std::ffi::OsStr;
use std::num::NonZeroU32;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
slint::include_modules!();
use log::error;

use crate::basic_physics_system::BasicPhysicsSystem;
use crate::camera::Camera;
use crate::camera_input::{
    apply_camera_input, update_camera_messages, ActiveCamera, CameraInputMessage, CameraInputState,
};
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

type SharedMeshRenderer = Rc<RefCell<Option<Renderer>>>;
type SharedMouseState = Rc<RefCell<MouseState>>;
type SharedSettings = Arc<Mutex<Settings>>;
type SharedActionManager = Arc<Mutex<ActionManager>>;

#[allow(dead_code)]
struct AppState {
    mouse_state: SharedMouseState,
    shared_renderer: SharedMeshRenderer,
    shared_settings: SharedSettings,
    shared_action_manager: SharedActionManager,
}

fn main() {
    std::env::set_var("SLINT_BACKEND", "GL");
    // Initialize the Slint application
    let app = App::new().unwrap();
    let app_weak = app.as_weak();
    let settings = Settings::load_user_settings();

    let world = Rc::new(RefCell::new(World::new()));
    world
        .borrow_mut()
        .insert_resource(MeshResource::default());
    world.borrow_mut().insert_resource(RenderQueue::default());
    world.borrow_mut().insert_resource(RenderResourceManager::new());
    world
        .borrow_mut()
        .insert_resource(CameraInputState::default());
    world.borrow_mut().insert_resource(ActiveCamera::default());
    world
        .borrow_mut()
        .init_resource::<Messages<CameraInputMessage>>();

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

    let ecs_timer = Timer::default();
    {
        let world = world.clone();
        let schedule = schedule.clone();

        ecs_timer.start(
            slint::TimerMode::Repeated,
            std::time::Duration::from_millis(16), // ~60 Hz
            move || {
                schedule.borrow_mut().run(&mut world.borrow_mut());
            },
        );
    }

    let state = AppState {
        mouse_state: Rc::new(RefCell::new(MouseState::default())),
        shared_renderer: Rc::new(RefCell::new(None)),
        shared_settings: settings.clone(),
        shared_action_manager: Arc::new(Mutex::new(ActionManager::new())),
    };

    {
        // Set the rendering notifier with a closure
        // Create a weak reference to the app for use inside the closure
        let app_weak_clone = app_weak.clone(); // Clone app_weak for use inside the closure
        let renderer_clone = Rc::clone(&state.shared_renderer);
        let shared_settings = Arc::clone(&state.shared_settings);
        let world = world.clone();
        if let Err(error) = app.window().set_rendering_notifier({
            move |rendering_state, graphics_api| {
                match rendering_state {
                    slint::RenderingState::RenderingSetup => {
                        // Initialize OpenGL context
                        let gl: GlowContext = match graphics_api {
                            slint::GraphicsAPI::NativeOpenGL { get_proc_address } => unsafe {
                                GlowContext::from_loader_function_cstr(|s| get_proc_address(s))
                            },
                            _ => panic!("Unsupported Graphics API"),
                        };
                        let gl = Rc::new(gl); // Wrap in Rc for shared ownership
                        let version = unsafe { gl.get_parameter_string(glow::VERSION) };
                        println!("OpenGL Version: {}", version);

                        let shading_language_version =
                            unsafe { gl.get_parameter_string(glow::SHADING_LANGUAGE_VERSION) };
                        println!("GLSL Version: {}", shading_language_version);

                        let major_version = unsafe { gl.get_parameter_i32(glow::MAJOR_VERSION) };
                        let minor_version = unsafe { gl.get_parameter_i32(glow::MINOR_VERSION) };
                        println!(
                            "OpenGL Major Version: {}. OpenGL Minor Version: {}",
                            major_version, minor_version
                        );
                        let render_scale = shared_settings.lock().unwrap().renderer.render_scale;
                        // Initialize renderer
                        let renderer = Renderer::new(
                            gl.clone(),
                            (1600.0 * render_scale) as u32,
                            (900.0 * render_scale) as u32,
                        );
                        *renderer_clone.borrow_mut() = Some(renderer);

                        let w = &mut world.borrow_mut();
                        let camera_handle = w
                            .get_resource_mut::<RenderResourceManager>()
                            .unwrap()
                            .camera_manager
                            .add_camera(Camera::new(16.0 / 9.0));
                        if let Some(mut active_camera) = w.get_resource_mut::<ActiveCamera>() {
                            active_camera.0 = camera_handle;
                        } else {
                            w.insert_resource(ActiveCamera(camera_handle));
                        }

                        let test_gltfs = [
                            "resources/models/cube/Cube.gltf",
                            "resources/models/normal_tangent_test/NormalTangentMirrorTest.gltf",
                            "resources/models/suzanne/Suzanne.gltf",
                        ];

                        let test_objects = {
                            let mut render_data_manager =
                                w.get_resource_mut::<RenderResourceManager>().unwrap();

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

                                let m_handle =
                                    material_handles.pop().expect("No materials found in glTF");

                                objects.push((mesh_handle, m_handle));
                            }
                            objects
                        };

                        let t_range = 2.0;
                        for (mesh_handle, material_handle) in test_objects {
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
                            w.spawn((
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
                                    mesh_id: mesh_handle,
                                },
                                MaterialComponent {
                                    material_id: material_handle,
                                },
                            ));
                        }
                    }
                    slint::RenderingState::BeforeRendering => {
                        // Access the renderer
                        if let Some(renderer) = renderer_clone.borrow_mut().as_mut() {
                            if let Some(app) = app_weak_clone.upgrade() {
                                // Get the requested texture size for current window size
                                let height = app.get_requested_texture_height() as f32;
                                let width = app.get_requested_texture_width() as f32;
                                let renderer_settings = &shared_settings.lock().unwrap().renderer;
                                let render_scale = renderer_settings.render_scale;
                                let render_params = RenderParams {
                                    width: (width * render_scale) as u32,
                                    height: (height * render_scale) as u32,
                                    visualize_edges: renderer_settings.visualize_edges,
                                    visualize_normals: renderer_settings.visualize_normals,
                                };
                                let w = &mut world.borrow_mut();
                                // 1. Extract instance data FIRST
                                let instances: Vec<RenderInstance> = {
                                    let render_queue = w
                                        .get_resource::<RenderQueue>()
                                        .expect("RenderQueue resource not found");
                                    render_queue.instances.clone()
                                };

                                // 2. Get the render data manager
                                let mut render_data_manager = w
                                    .get_resource_mut::<RenderResourceManager>()
                                    .expect("RenderDataManager resource not found");

                                // 3. Render
                                let texture = renderer.render(
                                    render_params,
                                    &mut *render_data_manager,
                                    instances,
                                );

                                app.set_texture(texture);
                                app.set_visualize_edges(renderer_settings.visualize_edges);
                                app.set_visualize_normals(renderer_settings.visualize_normals);

                                app.window().request_redraw();
                            }
                        }
                    }
                    slint::RenderingState::AfterRendering => {
                        // Optional: Perform any post-rendering tasks
                    }
                    slint::RenderingState::RenderingTeardown => {
                        println!["Rendering teardown"];
                        // Clean up the renderer

                        *renderer_clone.borrow_mut() = None;
                    }
                    _ => {}
                }
            }
        }) {
            match error {
            slint::SetRenderingNotifierError::Unsupported => eprintln!(
                "This game requires the use of the GL backend. Please run with the environment variable SLINT_BACKEND=GL set."
            ),
            _ => unreachable!(),
        }
            std::process::exit(1);
        }
    }

    // Handler for scrollwheel/scroll gesture
    {
        let world = world.clone();
        app.on_mouse_scroll(move |amt| {
            if let Ok(mut w) = world.try_borrow_mut() {
                let _ = w.write_message(CameraInputMessage::MouseScroll { delta: amt });
            } else {
                debug!("Skipping scroll input: world already borrowed");
            }
        });
    }

    // Handler for mouse movement
    {
        let world = world.clone();
        app.on_mouse_move_renderer(move |x, y| {
            if let Ok(mut w) = world.try_borrow_mut() {
                let _ = w.write_message(CameraInputMessage::MouseMove { x, y: y });
            } else {
                debug!("Skipping mouse move input: world already borrowed");
            }
        });
    }

    // Mouse down handler
    {
        let world = world.clone();
        app.on_mouse_down_renderer(move |button| {
            if let Ok(mut w) = world.try_borrow_mut() {
                let _ = w.write_message(CameraInputMessage::MouseDown {
                    button: button.into(),
                });
            } else {
                debug!("Skipping mouse down input: world already borrowed");
            }
        });
    }
    // Mouse up handler
    {
        let world = world.clone();
        app.on_mouse_up_renderer(move |button| {
            if let Ok(mut w) = world.try_borrow_mut() {
                let _ = w.write_message(CameraInputMessage::MouseUp {
                    button: button.into(),
                });
            } else {
                debug!("Skipping mouse up input: world already borrowed");
            }
        });
    }

    // Onclick handlers for the visualization options buttons
    {
        let shared_settings = Arc::clone(&state.shared_settings);
        app.on_toggle_edge_visualization(move || {
            let mut mg = shared_settings.lock().unwrap();
            let v = mg.renderer.visualize_edges;
            mg.renderer.visualize_edges = !v;

            match mg.save_user_settings() {
                Ok(_) => println!("User settings updated"),
                Err(e) => error!("Error when updating user settings: {:?}", e),
            }
        });

        let shared_settings = Arc::clone(&state.shared_settings);
        app.on_toggle_normal_visualization(move || {
            let mut mg = shared_settings.lock().unwrap();
            let v = mg.renderer.visualize_normals;
            mg.renderer.visualize_normals = !v;

            match mg.save_user_settings() {
                Ok(_) => println!("User settings updated"),
                Err(e) => error!("Error when updating user settings: {:?}", e),
            }
        });
    }

    // Run the Slint application
    app.run().unwrap();
}

macro_rules! define_scoped_binding {
    (struct $binding_ty_name:ident => $obj_name:path, $param_name:path, $binding_fn:ident, $target_name:path) => {
        struct $binding_ty_name {
            saved_value: Option<$obj_name>,
            gl: Rc<GlowContext>,
        }

        impl $binding_ty_name {
            unsafe fn new(gl: &Rc<GlowContext>, new_binding: Option<$obj_name>) -> Self {
                let saved_value =
                    NonZeroU32::new(gl.get_parameter_i32($param_name) as u32).map($obj_name);

                gl.$binding_fn($target_name, new_binding);
                Self {
                    saved_value,
                    gl: gl.clone(),
                }
            }
        }

        impl Drop for $binding_ty_name {
            fn drop(&mut self) {
                unsafe {
                    self.gl.$binding_fn($target_name, self.saved_value);
                }
            }
        }
    };
    (struct $binding_ty_name:ident => $obj_name:path, $param_name:path, $binding_fn:ident) => {
        struct $binding_ty_name {
            saved_value: Option<$obj_name>,
            gl: Rc<GlowContext>,
        }

        impl $binding_ty_name {
            unsafe fn new(gl: &Rc<GlowContext>, new_binding: Option<$obj_name>) -> Self {
                let saved_value =
                    NonZeroU32::new(gl.get_parameter_i32($param_name) as u32).map($obj_name);

                gl.$binding_fn(new_binding);
                Self {
                    saved_value,
                    gl: gl.clone(),
                }
            }
        }

        impl Drop for $binding_ty_name {
            fn drop(&mut self) {
                unsafe {
                    self.gl.$binding_fn(self.saved_value);
                }
            }
        }
    };
}

// define_scoped_binding!(struct ScopedTextureBinding => glow::NativeTexture, glow::TEXTURE_BINDING_2D, bind_texture, glow::TEXTURE_2D);
define_scoped_binding!(struct ScopedFrameBufferBinding => glow::NativeFramebuffer, glow::DRAW_FRAMEBUFFER_BINDING, bind_framebuffer, glow::DRAW_FRAMEBUFFER);
