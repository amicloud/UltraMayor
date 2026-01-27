use std::rc::Rc;
slint::include_modules!();
use crate::camera::Camera;
use crate::handles::MaterialHandle;
use crate::handles::MeshHandle;
use crate::mesh::Mesh;
use crate::render_data_manager::RenderDataManager;
use crate::render_instance::RenderInstance;
use crate::render_texture::RenderTexture;
use glow::Context as GlowContext;
use glow::HasContext;
use nalgebra::Vector3;
pub struct Renderer {
    gl: Rc<GlowContext>,
    displayed_texture: RenderTexture,
    next_texture: RenderTexture,
    camera: Camera,
    frames_rendered: u64,
}
pub struct RenderParams {
    pub width: u32,
    pub height: u32,
    pub visualize_edges: bool,
    pub visualize_normals: bool,
}
impl Renderer {
    pub fn render(
        &mut self,
        render_params: RenderParams,
        render_data_manager: &mut RenderDataManager,
        instances: Vec<RenderInstance>,
    ) -> slint::Image {
        unsafe {
            let gl = &self.gl;
            let current_time = std::time::Instant::now();
            let mut draw_calls = 0;

            // Resize FBO texture if needed
            if self.next_texture.width != render_params.width
                || self.next_texture.height != render_params.height
            {
                println!("Resizing and making new? fbo texture");
                let mut new_tex = RenderTexture::new(gl, render_params.width, render_params.height);
                std::mem::swap(&mut self.next_texture, &mut new_tex);
            }

            self.next_texture.with_texture_as_active_fbo(|| {
                gl.enable(glow::DEPTH_TEST);
                gl.depth_func(glow::LEQUAL);
                gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
                gl.enable(glow::CULL_FACE);
                gl.cull_face(glow::BACK);

                let saved_viewport = {
                    let mut vp = [0; 4];
                    gl.get_parameter_i32_slice(glow::VIEWPORT, &mut vp);
                    vp
                };

                gl.viewport(
                    0,
                    0,
                    self.next_texture.width as i32,
                    self.next_texture.height as i32,
                );

                // Update camera
                self.camera
                    .set_aspect_ratio(render_params.width as f32 / render_params.height as f32);
                let view_proj = self.camera.projection_matrix * self.camera.view_matrix();

                // ------------------------------------------------------------
                // PASS 1: Frustum culling
                // ------------------------------------------------------------
                let frustum = crate::frustum::Frustum::from_view_proj(&view_proj);

                let mut visible_instances: Vec<&RenderInstance> = Vec::new();

                for inst in &instances {
                    // println!("Checking culling for instance: {:?}", inst);
                    let mesh = render_data_manager
                        .mesh_manager
                        .get_mesh(inst.mesh_id)
                        .expect("Mesh not found");

                    let scale = inst.transform.fixed_view::<3, 3>(0, 0).abs().max();
                    let world_center = inst
                        .transform
                        .transform_point(&nalgebra::Point3::from(mesh.sphere_center));
                    let world_radius = mesh.sphere_radius * scale;

                    if frustum.intersects_sphere(world_center.coords, world_radius) {
                        visible_instances.push(inst);
                    }
                }

                // ------------------------------------------------------------
                // PASS 2: Group by material
                // ------------------------------------------------------------
                use std::collections::HashMap;
                let mut instances_by_material: HashMap<MaterialHandle, Vec<&RenderInstance>> =
                    HashMap::new();
                for inst in &visible_instances {
                    instances_by_material
                        .entry(inst.material_id)
                        .or_default()
                        .push(inst);
                }

                // ------------------------------------------------------------
                // PASS 3: Render
                // ------------------------------------------------------------
                let default_light_color = Vector3::new(1.0, 1.0, 1.0);
                for (material_id, inst_group) in instances_by_material {
                    let material = render_data_manager
                        .material_manager
                        .get_material(material_id)
                        .expect("Material not found");
                    let shader = &material.desc.shader;

                    // Bind shader
                    gl.use_program(Some(shader.program));

                    // Set common uniforms
                    let view_proj_matrix: [f32; 16] = view_proj.as_slice().try_into().unwrap();
                    gl.uniform_matrix_4_f32_slice(
                        Some(&shader.u_view_proj_location),
                        false,
                        &view_proj_matrix,
                    );
                    gl.uniform_3_f32(
                        Some(&shader.u_camera_position_location),
                        self.camera.position.x,
                        self.camera.position.y,
                        self.camera.position.z,
                    );

                    // Material-specific uniforms
                    gl.uniform_1_f32(
                        Some(&shader.u_roughness_location),
                        f32::from_bits(material.desc.roughness),
                    );
                    gl.uniform_1_f32(
                        Some(&shader.u_base_reflectance_location),
                        f32::from_bits(material.desc.base_reflectance),
                    );

                    // Light uniforms
                    gl.uniform_3_f32(Some(&shader.u_light_direction_location), 0.0, 0.0, 1.0);
                    gl.uniform_3_f32(
                        Some(&shader.u_light_color_location),
                        default_light_color.x,
                        default_light_color.y,
                        default_light_color.z,
                    );

                    // Visualization uniforms
                    gl.uniform_1_f32(
                        Some(&shader.u_visualize_normals_location),
                        if render_params.visualize_normals {
                            1.0
                        } else {
                            0.0
                        },
                    );
                    gl.uniform_1_f32(
                        Some(&shader.u_visualize_edges_location),
                        if render_params.visualize_edges {
                            1.0
                        } else {
                            0.0
                        },
                    );
                    gl.uniform_1_f32(Some(&shader.u_edge_thickness_location), 1.0);

                    // Bind textures
                    let albedo_tex = material.desc.albedo;
                    let tex = render_data_manager
                        .texture_manager
                        .get_texture(albedo_tex)
                        .expect("Albedo texture missing");
                    gl.active_texture(glow::TEXTURE0);
                    gl.bind_texture(glow::TEXTURE_2D, tex.gl_tex);
                    gl.uniform_1_i32(Some(&shader.u_albedo_location), 0);

                    // Optional normal map
                    let normal_tex_handle = material
                        .desc
                        .normal
                        .unwrap_or(render_data_manager.texture_manager.default_normal_map);

                    let tex = render_data_manager
                        .texture_manager
                        .get_texture(normal_tex_handle)
                        .expect("Normal texture missing");
                    gl.active_texture(glow::TEXTURE1);
                    gl.bind_texture(glow::TEXTURE_2D, tex.gl_tex);
                    gl.uniform_1_i32(Some(&shader.u_normal_location), 1);

                    // Group by mesh
                    let mut instances_by_mesh: HashMap<MeshHandle, Vec<[f32; 16]>> = HashMap::new();
                    for inst in inst_group {
                        instances_by_mesh
                            .entry(inst.mesh_id)
                            .or_default()
                            .push(inst.transform.as_slice().try_into().unwrap());
                    }

                    // Draw each mesh
                    for (mesh_id, matrices) in instances_by_mesh {
                        let mesh = render_data_manager
                            .mesh_manager
                            .get_mesh_mut(mesh_id)
                            .expect("Mesh not found");
                        mesh.update_instance_buffer(&matrices, gl);

                        gl.bind_vertex_array(mesh.vao);
                        gl.draw_elements_instanced(
                            glow::TRIANGLES,
                            mesh.indices.len() as i32,
                            glow::UNSIGNED_INT,
                            0,
                            matrices.len() as i32,
                        );
                        draw_calls += 1;
                    }

                    // Unbind textures after drawing material
                    gl.active_texture(glow::TEXTURE0);
                    gl.bind_texture(glow::TEXTURE_2D, None);
                    gl.active_texture(glow::TEXTURE1);
                    gl.bind_texture(glow::TEXTURE_2D, None);
                }

                gl.bind_vertex_array(None);
                gl.viewport(
                    saved_viewport[0],
                    saved_viewport[1],
                    saved_viewport[2],
                    saved_viewport[3],
                );
            });

            // Swap FBO textures for Slint
            let result_texture = slint::BorrowedOpenGLTextureBuilder::new_gl_2d_rgba_texture(
                self.next_texture.texture.0,
                (self.next_texture.width, self.next_texture.height).into(),
            )
            .build();
            std::mem::swap(&mut self.next_texture, &mut self.displayed_texture);

            self.frames_rendered += 1;
            if self.frames_rendered % 60 == 0 {
                println!("Frames rendered: {}", self.frames_rendered);
                println!(
                    "Render time: {:.2} ms",
                    current_time.elapsed().as_secs_f32() * 1000.0
                );
                println!("Draw calls on last frame: {}", draw_calls);
            }

            result_texture
        }
    }

    pub fn new(gl: Rc<GlowContext>, width: u32, height: u32) -> Self {
        unsafe {
            let aspect_ratio = width as f32 / height as f32;
            let camera = Camera::new(aspect_ratio);

            let depth_buffer = gl.create_renderbuffer().unwrap();
            gl.bind_renderbuffer(glow::RENDERBUFFER, Some(depth_buffer));
            gl.framebuffer_renderbuffer(
                glow::FRAMEBUFFER,
                glow::DEPTH_ATTACHMENT,
                glow::RENDERBUFFER,
                Some(depth_buffer),
            );

            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);

            // Initialize textures
            let displayed_texture = RenderTexture::new(&gl, width, height);
            let next_texture = RenderTexture::new(&gl, width, height);
            let renderer = Self {
                gl,
                // model_location,
                displayed_texture,
                next_texture,
                camera,
                frames_rendered: 0,
            };
            renderer
        }
    }

    pub fn camera_pitch_yaw(&mut self, delta_x: f32, delta_y: f32) {
        self.camera.pitch_yaw(delta_x, -delta_y);
    }

    pub fn camera_pan(&mut self, delta_x: f32, delta_y: f32) {
        self.camera.pan(delta_x, delta_y);
    }

    pub(crate) fn zoom(&mut self, amt: f32) {
        self.camera.zoom(amt);
    }

    /// Deletes a mesh's GPU resources
    #[allow(dead_code)]
    pub fn delete_mesh_gpu(&self, mesh: &mut Mesh) {
        unsafe {
            if let Some(vao) = mesh.vao.take() {
                self.gl.delete_vertex_array(vao);
            }
            if let Some(vbo) = mesh.vbo.take() {
                self.gl.delete_buffer(vbo);
            }
            if let Some(ebo) = mesh.ebo.take() {
                self.gl.delete_buffer(ebo);
            }
            if let Some(inst) = mesh.instance_vbo.take() {
                self.gl.delete_buffer(inst);
            }
            mesh.instance_count = 0;
        }
    }
}
