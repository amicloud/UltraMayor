// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

mod body;
mod camera;
mod cpu_slicer;
mod gpu_slicer;
mod mesh;
mod mesh_renderer;
mod render_texture;
mod stl_processor;
use action_manager::ActionManager;
use body::Body;
use cpu_slicer::{CPUSlicer, CPUSlicerError};
use glow::Context as GlowContext;
use glow::HasContext;
use image::{ImageBuffer, Luma};
use log::debug;
use mesh_renderer::MeshRenderer;
use nalgebra::Vector3;
use printer::Printer;
use rfd::AsyncFileDialog;
use settings::Settings;
use slint::platform::PointerEventButton;
use slint::SharedString;
use tokio::sync::mpsc::error;
use std::cell::RefCell;
use std::num::NonZeroU32;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use stl_processor::StlProcessor;
use tokio::task;
mod file_manager;
mod mesh_island_analyzer;
use crate::file_manager::file_manager::write_webps_to_folder;
use mesh_island_analyzer::MeshIslandAnalyzer;
slint::include_modules!();
mod action;
mod action_manager;
mod material;
mod printer;
mod settings;
use crate::action::{SetPositionAction, SetRotationAction, SetScaleAction};
use log::error;
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

type SharedBodies = Rc<RefCell<Vec<Rc<RefCell<Body>>>>>;
type SharedMeshRenderer = Rc<RefCell<Option<MeshRenderer>>>;
type SharedMouseState = Rc<RefCell<MouseState>>;
type SharedSettings = Arc<Mutex<Settings>>;
type SharedPrinter = Arc<Mutex<Printer>>;
type SharedActionManager = Arc<Mutex<ActionManager>>;

struct AppState {
    mouse_state: SharedMouseState,
    shared_mesh_renderer: SharedMeshRenderer,
    shared_bodies: SharedBodies,
    shared_settings: SharedSettings,
    shared_printer: SharedPrinter,
    shared_action_manager: SharedActionManager,
}


fn main() {
    // Initialize the Slint application
    let app = App::new().unwrap();
    let app_weak = app.as_weak();
    let settings = Settings::load_user_settings();

    let state = AppState {
        mouse_state: Rc::new(RefCell::new(MouseState::default())),
        shared_mesh_renderer: Rc::new(RefCell::new(None)),
        shared_bodies: Rc::new(RefCell::new(Vec::<Rc<RefCell<Body>>>::new())), // Initialized as empty Vec
        shared_settings: settings.clone(),
        shared_printer: Arc::new(Mutex::new(Printer::default())),
        shared_action_manager: Arc::new(Mutex::new(ActionManager::new())),
        
    };

    {
        // Set the rendering notifier with a closure
        // Create a weak reference to the app for use inside the closure
        let app_weak_clone = app_weak.clone(); // Clone app_weak for use inside the closure
        let mesh_renderer_clone = Rc::clone(&state.shared_mesh_renderer);
        let bodies_clone = Rc::clone(&state.shared_bodies);
        let shared_printer = Arc::clone(&state.shared_printer);
        let shared_settings = Arc::clone(&state.shared_settings);
        if let Err(error) = app.window().set_rendering_notifier({
            // Move clones into the closure
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
                        let gl = Rc::new(gl); // Wrap in Rc

                        // Use 'gl' to get OpenGL version strings etc.
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
                        // Initialize renderer and slicers with cloned Rc
                        let renderer = MeshRenderer::new(
                            gl.clone(),
                            (1000.0 * render_scale) as u32,
                            (1000.0 * render_scale) as u32,
                            &bodies_clone,
                            &shared_printer.clone(),
                        );
                        *mesh_renderer_clone.borrow_mut() = Some(renderer);
                    }
                    slint::RenderingState::BeforeRendering => {
                        // Access the renderer
                        if let Some(renderer) = mesh_renderer_clone.borrow_mut().as_mut() {
                            if let Some(app) = app_weak_clone.upgrade() {
                                let height = app.get_requested_texture_height() as f32;
                                let width = app.get_requested_texture_width() as f32;
                                let renderer_settings = 
                                &shared_settings.lock().unwrap().renderer;
                                let render_scale =
                                renderer_settings.render_scale;
                                let texture = renderer.render(
                                    (width * render_scale) as u32,
                                    (height * render_scale) as u32,
                                    renderer_settings.visualize_edges,
                                    renderer_settings.visualize_normals,
                                );

                                let mut bodies_ui_vec: Vec<BodyUI> = Vec::new();
                                let mut num_bodies = 0;
                                for body in bodies_clone.borrow_mut().iter() {
                                    if !body.borrow().display_in_ui_list {
                                        continue;
                                    }
                                    num_bodies += 1;
                                    let b = body.borrow_mut();
                                    bodies_ui_vec.push(BodyUI {
                                        enabled: b.enabled,
                                        name: b.name.clone().into(),
                                        uuid: b.uuid.clone().to_string().into(),
                                        visible: b.visible,
                                        selected: b.selected,
                                        p_x: b.position.x.to_string().clone().into(),
                                        p_y: b.position.y.to_string().clone().into(),
                                        p_z: b.position.z.to_string().clone().into(),
                                        r_x: b.rotation.i.to_string().clone().into(),
                                        r_y: b.rotation.j.to_string().clone().into(),
                                        r_z: b.rotation.k.to_string().clone().into(),
                                        s_x: b.scale.x.to_string().clone().into(),
                                        s_y: b.scale.y.to_string().clone().into(),
                                        s_z: b.scale.z.to_string().clone().into(),
                                    })
                                }

                                let bodies_model: Rc<slint::VecModel<BodyUI>> =
                                    std::rc::Rc::new(slint::VecModel::from(bodies_ui_vec));

                                // Update UI model
                                app.set_bodies(bodies_model.into());
                                app.set_num_bodies(num_bodies);
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
                        // Clean up the renderer
                        *mesh_renderer_clone.borrow_mut() = None;
                    }
                    _ => {}
                }
            }
        }) {
            match error {
            slint::SetRenderingNotifierError::Unsupported => eprintln!(
                "This example requires the use of the GL backend. Please run with the environment variable SLINT_BACKEND=GL set."
            ),
            _ => unreachable!(),
        }
            std::process::exit(1);
        }
    }

    // Handler for scrollwheel zooming TODO: Consider renaming for clarity
    {
        let app_weak_clone = app_weak.clone(); // Clone app_weak again for this closure
        let mesh_renderer_clone = Rc::clone(&state.shared_mesh_renderer); // Clone mesh_renderer for this closure
        app.on_zoom(move |amt| {
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
        let mesh_renderer_clone = Rc::clone(&state.shared_mesh_renderer); // Clone mesh_renderer for this closure
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

    async fn open_files_from_dialog(bodies_clone: &SharedBodies) {
        // Handling the option prevents crashes
        if let Some(paths) = AsyncFileDialog::new()
            .add_filter("stl", &["stl", "STL"])
            .set_directory("~")
            .pick_files()
            .await
        {
            let stl_processor = StlProcessor::new();
            let mut bodies_vec: Vec<Rc<RefCell<Body>>> = Vec::new();

            for path in paths {
                let body = Rc::new(RefCell::new(Body::new_from_stl(
                    path.path().as_os_str(),
                    &stl_processor,
                )));
                bodies_vec.push(Rc::clone(&body));
                println!("Loaded body: {}", path.file_name());
            }
            bodies_clone.borrow_mut().append(&mut bodies_vec);
        } else {
            println!("File picker returned no files");
        }
    }

    // Handler for opening STL importer file picker
    {
        let bodies_clone = Rc::clone(&state.shared_bodies);
        app.on_click_import_stl(move || {
            let bc_clone = Rc::clone(&bodies_clone);
            let slint_future = async move {
                open_files_from_dialog(&bc_clone).await;
            };
            slint::spawn_local(async_compat::Compat::new(slint_future)).unwrap();
        });
    }

    // Handlers for objectlistitem editing
    {
        let bodies_clone = Rc::clone(&state.shared_bodies);
        let action_manager = Arc::clone(&state.shared_action_manager);
        app.on_body_position_edited_single_axis(
            move |uuid: slint::SharedString, amt: f32, axis: i32| {
                let bodies = bodies_clone.borrow();

                // Find the body to modify
                if let Some(body_rc) = bodies
                    .iter()
                    .find(|body_rc| body_rc.borrow().eq_uuid_ss(&uuid))
                {
                    // Determine the new position vector
                    let new_position = {
                        let body = body_rc.borrow(); // Immutable borrow to access position
                        match axis {
                            0 => Vector3::new(amt, body.position.y, body.position.z),
                            1 => Vector3::new(body.position.x, amt, body.position.z),
                            2 => Vector3::new(body.position.x, body.position.y, amt),
                            _ => Vector3::default(),
                        }
                    }; // Dropping `body` here after usage

                    // Set up the action
                    let previous_position = body_rc.borrow().position;
                    let action = SetPositionAction {
                        body: body_rc.clone(),
                        input: new_position,
                        previous: previous_position,
                    };

                    // Execute the action via ActionManager
                    let mut manager = action_manager.lock().unwrap();
                    manager.execute(Box::new(action));
                }
            },
        );

        let bodies_clone = Rc::clone(&state.shared_bodies);
        let action_manager = Arc::clone(&state.shared_action_manager);
        app.on_body_rotation_edited_single_axis(
            move |uuid: slint::SharedString, amt: f32, axis: i32| {
                let bodies = bodies_clone.borrow();

                // Find the body to modify
                if let Some(body_rc) = bodies
                    .iter()
                    .find(|body_rc| body_rc.borrow().eq_uuid_ss(&uuid))
                {
                    // Calculate new rotation vector without holding mutable borrow
                    let new_rotation = {
                        let body = body_rc.borrow(); // Immutable borrow for accessing rotation
                        let rotation = Body::quaternion_to_euler(&body.rotation);
                        match axis {
                            0 => Vector3::new(amt, rotation.y, rotation.z),
                            1 => Vector3::new(rotation.x, amt, rotation.z),
                            2 => Vector3::new(rotation.x, rotation.y, amt),
                            _ => Vector3::default(),
                        }
                    }; // Borrow ends here

                    // Set up the action with non-overlapping borrows
                    let action = SetRotationAction {
                        body: body_rc.clone(),
                        input: new_rotation,
                        previous: body_rc.borrow().rotation, // Immutable borrow for previous value
                    };

                    // Execute the action via ActionManager
                    let mut manager = action_manager.lock().unwrap();
                    manager.execute(Box::new(action));
                }
            },
        );

        let bodies_clone = Rc::clone(&state.shared_bodies);
        let action_manager = Arc::clone(&state.shared_action_manager);
        app.on_body_scale_edited_single_axis(
            move |uuid: slint::SharedString, amt: f32, axis: i32| {
                let bodies = bodies_clone.borrow();

                // Find the body to modify
                if let Some(body_rc) = bodies
                    .iter()
                    .find(|body_rc| body_rc.borrow().eq_uuid_ss(&uuid))
                {
                    // Calculate new scale vector without holding mutable borrow
                    let new_scale = {
                        let body = body_rc.borrow(); // Immutable borrow for accessing scale
                        match axis {
                            0 => Vector3::new(amt, body.scale.y, body.scale.z),
                            1 => Vector3::new(body.scale.x, amt, body.scale.z),
                            2 => Vector3::new(body.scale.x, body.scale.y, amt),
                            _ => Vector3::default(),
                        }
                    }; // Borrow ends here

                    // Set up the action with non-overlapping borrows
                    let action = SetScaleAction {
                        body: body_rc.clone(),
                        input: new_scale,
                        previous: body_rc.borrow().scale, // Immutable borrow for previous value
                    };

                    // Execute the action via ActionManager
                    let mut manager = action_manager.lock().unwrap();
                    manager.execute(Box::new(action));
                }
            },
        );

        let bodies_clone = Rc::clone(&state.shared_bodies);
        app.on_toggle_body_selected(move |uuid| {
            for body_rc in bodies_clone.borrow().iter() {
                let mut body = body_rc.borrow_mut();
                if body.eq_uuid_ss(&uuid) {
                    body.selected = !body.selected;
                }
            }
        });
    }

    async fn slice_all_bodies(
        bodies_clone: SharedBodies,
    ) -> Result<Vec<ImageBuffer<Luma<u8>, Vec<u8>>>, CPUSlicerError> {
        // Borrow the bodies vector and copy the data
        let bodies: Vec<Body> = bodies_clone
            .borrow()
            .iter()
            .map(|b| b.borrow().clone())
            .collect();
        // Offload the CPU-intensive slicing to a blocking thread
        let handle = task::spawn_blocking(move || {
            CPUSlicer::slice_bodies(bodies, 0.10, &Printer::default())
        });

        // Await the result and map the JoinError to CPUSlicerError
        let inner_result = handle
            .await
            .map_err(|e| CPUSlicerError::ThreadJoinError(format!("Thread join error: {}", e)))?;

        // `inner_result` is now `Result<Vec<ImageBuffer<Luma<u8>, Vec<u8>>>, CPUSlicerError>`
        let output = inner_result?;

        let _ = write_webps_to_folder(&output).await; // TODO: handle this

        // Return the processed images for visualization
        Ok(output)
    }

    async fn slice_selected_bodies(
        bodies_clone: SharedBodies,
    ) -> Result<Vec<ImageBuffer<Luma<u8>, Vec<u8>>>, CPUSlicerError> {
        // Clone the shared bodies to avoid holding the lock during processing
        let bodies: Vec<Body> = {
            let bodies_ref = bodies_clone.borrow();
            bodies_ref
                .iter()
                .filter(|b| b.borrow().selected)
                .map(|b| b.borrow().clone())
                .collect()
        };
        // Offload the CPU-intensive slicing to a blocking thread
        let handle = task::spawn_blocking(move || {
            CPUSlicer::slice_bodies(bodies, 0.10, &Printer::default())
        });

        // Await the result and map the JoinError to CPUSlicerError
        let inner_result = handle
            .await
            .map_err(|e| CPUSlicerError::ThreadJoinError(format!("Thread join error: {}", e)))?;

        // `inner_result` is now `Result<Vec<ImageBuffer<Luma<u8>, Vec<u8>>>, CPUSlicerError>`
        let output = inner_result?;

        let _ = write_webps_to_folder(&output).await; // TODO: handle this

        // Return the processed images for visualization
        Ok(output)
    }

    // Slicing button callbacks
    {
        let bodies_clone = Rc::clone(&state.shared_bodies);
        app.on_slice_selected(move || {
            let bodies_clone = Rc::clone(&bodies_clone);
            let slint_future = async move {
                slice_selected_bodies(bodies_clone).await.unwrap();
            };
            slint::spawn_local(async_compat::Compat::new(slint_future)).unwrap();
        });

        let bodies_clone = Rc::clone(&state.shared_bodies);
        app.on_slice_all(move || {
            let bodies_clone = Rc::clone(&bodies_clone);
            let slint_future = async move { slice_all_bodies(bodies_clone).await };
            slint::spawn_local(async_compat::Compat::new(slint_future)).unwrap();
        });
    }

    // Delete item callback

    let bodies_clone: SharedBodies = Rc::clone(&state.shared_bodies);
    app.on_delete_item_by_uuid(move |uuid: SharedString| {
        delete_body_by_uuid(&bodies_clone, uuid);
    });

    fn delete_body_by_uuid(bodies_clone: &Rc<RefCell<Vec<Rc<RefCell<Body>>>>>, uuid: SharedString) {
        // Find the body to remove without mutably borrowing bodies_clone
        let body_to_remove = {
            let bodies = bodies_clone.borrow();
            bodies
                .iter()
                .find(|body_rc| {
                    let body = body_rc.borrow();
                    body.eq_uuid_ss(&uuid)
                })
                .cloned()
        };

        if let Some(body_rc) = body_to_remove {
            // Remove the body from bodies_clone
            let mut bodies = bodies_clone.borrow_mut();
            if let Some(pos) = bodies.iter().position(|x| Rc::ptr_eq(x, &body_rc)) {
                bodies.remove(pos);
            }
        }
    }

    // Onclick handler for vertex analysis button

    app.on_analyze_vertex_islands(move || {
        let bodies_clone = Rc::clone(&state.shared_bodies);
        let bodies = bodies_clone.borrow();
        for body_rc in bodies.iter() {
            let body = body_rc.borrow_mut();
            if body.selected {
                let islands = MeshIslandAnalyzer::analyze_islands(&body);
                println!("Islands vertices: {:?}", islands);
            }
        }
    });

    // Onclick handlers for undo and redo buttons
    {
        let action_manager = Arc::clone(&state.shared_action_manager);
        app.on_undo(move || {
            action_manager.lock().unwrap().undo();
        });

        let action_manager = Arc::clone(&state.shared_action_manager);
        app.on_redo(move || {
            action_manager.lock().unwrap().redo();
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
define_scoped_binding!(struct ScopedVBOBinding => glow::NativeBuffer, glow::ARRAY_BUFFER_BINDING, bind_buffer, glow::ARRAY_BUFFER);
define_scoped_binding!(struct ScopedVAOBinding => glow::NativeVertexArray, glow::VERTEX_ARRAY_BINDING, bind_vertex_array);
