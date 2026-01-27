// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

mod action;
mod action_manager;
mod basic_physics_system;
mod camera;
mod frustum;
mod handles;
mod material;
mod material_component;
mod material_resource_manager;
mod mesh;
mod mesh_component;
mod mesh_resource_manager;
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
use bevy_ecs::prelude::*;
use glow::Context as GlowContext;
use glow::HasContext;
use log::debug;
use nalgebra::Vector3;
use rand::random_range;
use renderer::Renderer;
use settings::Settings;
use slint::platform::PointerEventButton;
use slint::Timer;
use std::cell::RefCell;
use std::ffi::OsStr;
use std::num::NonZeroU32;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
slint::include_modules!();
use log::error;

use crate::basic_physics_system::BasicPhysicsSystem;
use crate::material::Material;
use crate::material::MaterialDesc;
use crate::material_component::MaterialComponent;
use crate::mesh::Mesh;
use crate::mesh_component::MeshComponent;
use crate::mesh_resource_manager::MeshResourceManager;
use crate::render_data_manager::RenderDataManager;
use crate::render_instance::RenderInstance;
use crate::render_queue::RenderQueue;
use crate::render_system::RenderSystem;
use crate::renderer::RenderParams;
use crate::shader::Shader;
use crate::transform_component::TransformComponent;
use crate::velocity_component::VelocityComponent;
#[derive(Default)]
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
        .insert_resource(MeshResourceManager::default());
    world.borrow_mut().insert_resource(RenderQueue::default());
    world.borrow_mut().insert_resource(RenderDataManager::new());

    let schedule = Rc::new(RefCell::new({
        let mut s = Schedule::default();
        s.add_systems((
            BasicPhysicsSystem::update,
            RenderSystem::extract_render_data,
        ));
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
                            (1000.0 * render_scale) as u32,
                            (1000.0 * render_scale) as u32,
                        );
                        *renderer_clone.borrow_mut() = Some(renderer);
                        let w = &mut world.borrow_mut();
                        let mut render_data_manager =
                            w.get_resource_mut::<RenderDataManager>().unwrap();

                        render_data_manager
                            .texture_manager
                            .create_default_normal_map(&gl);

                        let test_object_id = render_data_manager.mesh_manager.add_mesh(
                            Mesh::from_gltf(OsStr::new("resources/models/cube/Cube.gltf")).unwrap(),
                            &gl,
                        );

                        let albedo_id = render_data_manager.texture_manager.load_from_file(
                            &gl,
                            OsStr::new("resources/models/cube/Cube_BaseColor.png"),
                        );

                        let normal_id = render_data_manager.texture_manager.load_from_file(
                            &gl,
                            OsStr::new("resources/models/cube/Cube_Normal.png"),
                        );

                        let m_desc = MaterialDesc::new(
                            Shader::new(
                                &gl,
                                OsStr::new("resources/shaders/pbr.vert"),
                                OsStr::new("resources/shaders/pbr.frag"),
                            ),
                            0.5,
                            0.04,
                            albedo_id,
                            Some(normal_id),
                        );

                        let m_handle = render_data_manager
                            .material_manager
                            .add_material(Material::new(m_desc));

                        let t_range = 2.0;
                        for _ in 0..100 {
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

                            let scale = 1.0;
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
                                    mesh_id: test_object_id,
                                },
                                MaterialComponent {
                                    material_id: m_handle,
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
                                    .get_resource_mut::<RenderDataManager>()
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
        let app_weak_clone = app_weak.clone(); // Clone app_weak again for this closure
        let mesh_renderer_clone = Rc::clone(&state.shared_renderer); // Clone mesh_renderer for this closure
        app.on_mouse_scroll(move |amt| {
            // Access the renderer
            if let Some(renderer) = mesh_renderer_clone.borrow_mut().as_mut() {
                // Move the camera
                renderer.zoom(amt);

                // Trigger a redraw
                if let Some(app) = app_weak_clone.upgrade() {
                    app.window().request_redraw();
                }
            }
        });
    }

    // Handler for mouse movement in renderer
    {
        let app_weak_clone = app_weak.clone(); // Clone app_weak again for this closure
        let mesh_renderer_clone = Rc::clone(&state.shared_renderer); // Clone mesh_renderer for this closure
        let mouse_state_clone = Rc::clone(&state.mouse_state);
        app.on_mouse_move_renderer(move |x, y| {
            debug!("On mouse move event received");

            let mut mouse_state = mouse_state_clone.borrow_mut();

            // If the previous coords are still 0,0 then let's not move a bunch and return 0
            // This prevents some weird behavior, do not change
            let delta_x = x - if mouse_state.p_x != 0.0 {
                mouse_state.p_x
            } else {
                x
            };
            let delta_y = y - if mouse_state.p_y != 0.0 {
                mouse_state.p_y
            } else {
                y
            };

            mouse_state.p_x = x;
            mouse_state.p_y = y;
            mouse_state.x = x;
            mouse_state.y = y;
            debug!("Delta x: {:.3}, Delta y: {:.3}", delta_x, delta_y);
            debug!("Mouse pressed? {}", mouse_state.left_pressed);

            // Access the renderer
            if let Some(renderer) = mesh_renderer_clone.borrow_mut().as_mut() {
                if mouse_state.left_pressed {
                    renderer.camera_pitch_yaw(delta_x, delta_y);
                }
                if mouse_state.middle_pressed {
                    renderer.camera_pan(delta_x, delta_y);
                }
                // Trigger a redraw
                if let Some(app) = app_weak_clone.upgrade() {
                    app.window().request_redraw();
                }
            }
        });
    }

    // Mouse down handler for renderer
    {
        let mouse_state_clone = Rc::clone(&state.mouse_state);
        app.on_mouse_down_renderer(move |button| {
            debug!("On mouse down received");
            let mut mouse_state = mouse_state_clone.borrow_mut();
            match button {
                PointerEventButton::Left => mouse_state.left_pressed = true,
                PointerEventButton::Other => mouse_state.other_pressed = true,
                PointerEventButton::Right => mouse_state.right_pressed = true,
                PointerEventButton::Middle => mouse_state.middle_pressed = true,
                PointerEventButton::Back => mouse_state.back_pressed = true,
                PointerEventButton::Forward => mouse_state.forward_pressed = true,
                _ => {}
            }
        });
    }
    // Mouse up handler for renderer
    {
        let mouse_state_clone = Rc::clone(&state.mouse_state);
        app.on_mouse_up_renderer(move |button| {
            debug!("On mouse up received");
            let mut mouse_state = mouse_state_clone.borrow_mut();
            match button {
                PointerEventButton::Left => mouse_state.left_pressed = false,
                PointerEventButton::Other => mouse_state.other_pressed = false,
                PointerEventButton::Right => mouse_state.right_pressed = false,
                PointerEventButton::Middle => mouse_state.middle_pressed = false,
                PointerEventButton::Back => mouse_state.back_pressed = false,
                PointerEventButton::Forward => mouse_state.forward_pressed = false,
                _ => {}
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
