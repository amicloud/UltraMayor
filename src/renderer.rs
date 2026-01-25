use std::collections::HashMap;
use std::fs;
use std::rc::Rc;
slint::include_modules!();
use crate::camera::Camera;
use crate::material::Material;
use crate::mesh::Mesh;
use crate::mesh::Vertex;
use crate::mesh_resource_manager::MeshResourceManager;
use crate::render_instance::RenderInstance;
use crate::render_texture::RenderTexture;
use glow::Context as GlowContext;
use glow::HasContext;
use nalgebra::Vector3;
pub struct Renderer {
    gl: Rc<GlowContext>,
    program: glow::Program,

    view_proj_location: glow::UniformLocation,
    camera_position_location: glow::UniformLocation,
    // model_location: glow::UniformLocation,
    light_direction_location: glow::UniformLocation,
    light_color_location: glow::UniformLocation,
    albedo_location: glow::UniformLocation,
    roughness_location: glow::UniformLocation,
    base_reflectance_location: glow::UniformLocation,
    visualize_normals_location: glow::UniformLocation,
    visualize_edges_location: glow::UniformLocation,
    edge_thickness_location: glow::UniformLocation,

    displayed_texture: RenderTexture,
    next_texture: RenderTexture,
    camera: Camera,
}
pub struct RenderParams {
    pub width: u32,
    pub height: u32,
    pub visualize_edges: bool,
    pub visualize_normals: bool,
}
impl Renderer {
    pub fn new(gl: Rc<GlowContext>, width: u32, height: u32) -> Self {
        unsafe {
            // Create shader program
            let shader_program = gl.create_program().expect("Cannot create program");
            let aspect_ratio = width as f32 / height as f32;
            let camera = Camera::new(aspect_ratio);
            let manifest_dir = env!("CARGO_MANIFEST_DIR");
            let vertex_shader_path = format!("{}/resources/shaders/pbr.vert", manifest_dir);
            let fragment_shader_path = format!("{}/resources/shaders/pbr.frag", manifest_dir);

            let vertex_shader_source =
                fs::read_to_string(&vertex_shader_path).expect("Failed to read vertex shader file");
            let fragment_shader_source = fs::read_to_string(&fragment_shader_path)
                .expect("Failed to read fragment shader file");

            // Compile shaders and link program
            {
                let shader_sources = [
                    (glow::VERTEX_SHADER, vertex_shader_source),
                    (glow::FRAGMENT_SHADER, fragment_shader_source),
                ];

                let mut shaders = Vec::with_capacity(shader_sources.len());

                for (shader_type, shader_source) in &shader_sources {
                    let shader = gl
                        .create_shader(*shader_type)
                        .expect("Cannot create shader");
                    gl.shader_source(shader, shader_source);
                    gl.compile_shader(shader);
                    if !gl.get_shader_compile_status(shader) {
                        panic!(
                            "Fatal Error: Shader compile error: {}",
                            gl.get_shader_info_log(shader)
                        );
                    }
                    gl.attach_shader(shader_program, shader);
                    shaders.push(shader);
                }

                gl.link_program(shader_program);
                if !gl.get_program_link_status(shader_program) {
                    panic!(
                        "Fatal Error: Shader program link error: {}",
                        gl.get_program_info_log(shader_program)
                    );
                }

                for shader in shaders {
                    gl.detach_shader(shader_program, shader);
                    gl.delete_shader(shader);
                }
            }

            // Get attribute and uniform locations
            // Attributes
            let position_location: u32 =
                gl.get_attrib_location(shader_program, "position").unwrap();

            let normal_location: u32 = gl.get_attrib_location(shader_program, "normal").unwrap();

            let barycentric_location: u32 = gl
                .get_attrib_location(shader_program, "barycentric")
                .unwrap();

            // Uniforms
            let view_proj_location = gl
                .get_uniform_location(shader_program, "view_proj")
                .unwrap();

            let camera_position_location = gl
                .get_uniform_location(shader_program, "camera_position")
                .unwrap();

            // let model_location = gl.get_uniform_location(shader_program, "model").unwrap();

            let light_direction_location = gl
                .get_uniform_location(shader_program, "light_direction")
                .unwrap();

            let light_color_location = gl
                .get_uniform_location(shader_program, "light_color")
                .unwrap();

            let albedo_location = gl.get_uniform_location(shader_program, "albedo").unwrap();

            let roughness_location = gl
                .get_uniform_location(shader_program, "roughness")
                .unwrap();

            let base_reflectance_location = gl
                .get_uniform_location(shader_program, "base_reflectance")
                .unwrap();

            let visualize_normals_location = gl
                .get_uniform_location(shader_program, "visualize_normals")
                .unwrap();

            let visualize_edges_location = gl
                .get_uniform_location(shader_program, "visualize_edges")
                .unwrap();
            let edge_thickness_location = gl
                .get_uniform_location(shader_program, "edge_thickness")
                .unwrap();

            // Calculate the offsets for everything
            // much easier to read and reason about when laid out like this
            let vertex_stride: i32 = size_of::<Vertex>() as i32;
            let position_size = 3;
            let position_offset = 0;
            let normal_size = 3;
            let normal_offset = position_offset + position_size;
            let barycentric_size = 3;
            let barycentric_offset = normal_offset + normal_size;

            // Position attribute
            gl.enable_vertex_attrib_array(position_location);
            gl.vertex_attrib_pointer_f32(
                position_location,
                position_size,
                glow::FLOAT,
                false,
                vertex_stride,
                position_offset * 4,
            );

            // Normal attribute
            gl.enable_vertex_attrib_array(normal_location);
            gl.vertex_attrib_pointer_f32(
                normal_location,
                normal_size,
                glow::FLOAT,
                true,
                vertex_stride,
                normal_offset * 4,
            );

            // Barycentric attribute
            gl.enable_vertex_attrib_array(barycentric_location);
            gl.vertex_attrib_pointer_f32(
                barycentric_location,
                barycentric_size,
                glow::FLOAT,
                true,
                vertex_stride,
                barycentric_offset * 4,
            );

            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            gl.bind_vertex_array(None);

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
            let me = Self {
                gl,
                program: shader_program,
                view_proj_location,
                camera_position_location,
                light_direction_location,
                // model_location,
                displayed_texture,
                next_texture,
                camera,
                light_color_location,
                albedo_location,
                roughness_location,
                base_reflectance_location,
                visualize_normals_location,
                visualize_edges_location,
                edge_thickness_location,
            };
            me
        }
    }

    pub fn render(
        &mut self,
        render_params: RenderParams,
        meshes: &mut MeshResourceManager,
        instances: &[RenderInstance],
    ) -> slint::Image {
        unsafe {
            let gl = &self.gl;
            gl.use_program(Some(self.program));

            gl.enable(glow::CULL_FACE);
            gl.cull_face(glow::BACK);

            // Resize texture if necessary
            if self.next_texture.width != render_params.width
                || self.next_texture.height != render_params.height
            {
                let mut new_texture =
                    RenderTexture::new(gl, render_params.width, render_params.height);
                std::mem::swap(&mut self.next_texture, &mut new_texture);
            }

            let light_intensity = 0.25;
            let default_light_color =
                Vector3::new(light_intensity, light_intensity, light_intensity);

            self.next_texture.with_texture_as_active_fbo(|| {
                gl.enable(glow::DEPTH_TEST);
                gl.depth_func(glow::LEQUAL);
                gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);

                let mut saved_viewport = [0; 4];
                gl.get_parameter_i32_slice(glow::VIEWPORT, &mut saved_viewport);
                gl.viewport(
                    0,
                    0,
                    self.next_texture.width as i32,
                    self.next_texture.height as i32,
                );

                // Camera / matrices
                self.camera
                    .set_aspect_ratio(render_params.width as f32 / render_params.height as f32);

                let view_proj = self.camera.projection_matrix * self.camera.view_matrix();
                let view_proj_matrix: [f32; 16] = view_proj.as_slice().try_into().unwrap();

                gl.uniform_matrix_4_f32_slice(
                    Some(&self.view_proj_location),
                    false,
                    &view_proj_matrix,
                );

                gl.uniform_3_f32(
                    Some(&self.camera_position_location),
                    self.camera.position.x,
                    self.camera.position.y,
                    self.camera.position.z,
                );

                gl.uniform_3_f32(Some(&self.light_direction_location), 0.0, 0.0, 1.0);
                gl.uniform_3_f32(
                    Some(&self.light_color_location),
                    default_light_color.x,
                    default_light_color.y,
                    default_light_color.z,
                );

                // ------------------------------------------------------------
                // PASS 1: GATHER (no mesh mutation, no GL draws)
                // ------------------------------------------------------------
                let mut per_mesh_instances: HashMap<u32, Vec<[f32; 16]>> = HashMap::new();

                for inst in instances.iter() {
                    per_mesh_instances
                        .entry(inst.mesh_id)
                        .or_insert_with(Vec::new)
                        .push(inst.transform.as_slice().try_into().unwrap());
                }

                // ------------------------------------------------------------
                // PASS 2: UPLOAD + DRAW (mutable mesh access)
                // ------------------------------------------------------------
                let material = Material::default();

                gl.uniform_1_f32(Some(&self.roughness_location), material.roughness);
                gl.uniform_3_f32(
                    Some(&self.albedo_location),
                    material.albedo.x,
                    material.albedo.y,
                    material.albedo.z,
                );
                gl.uniform_3_f32(
                    Some(&self.base_reflectance_location),
                    material.base_reflectance.x,
                    material.base_reflectance.y,
                    material.base_reflectance.z,
                );

                gl.uniform_1_u32(
                    Some(&self.visualize_normals_location),
                    (render_params.visualize_normals && material.visualize_normals) as u32,
                );
                gl.uniform_1_u32(
                    Some(&self.visualize_edges_location),
                    (material.can_visualize_edges && render_params.visualize_edges) as u32,
                );
                gl.uniform_1_f32(Some(&self.edge_thickness_location), 3.0);
                // Rendering Loop 
                for (mesh_id, matrices) in per_mesh_instances {
                    let mesh = meshes.get_mesh_mut(mesh_id).expect("mesh not found");

                    // Upload per-instance transforms
                    mesh.update_instance_buffer(&matrices, gl);

                    gl.bind_vertex_array(mesh.vao);
                    gl.draw_elements_instanced(
                        glow::TRIANGLES,
                        mesh.indices.len() as i32,
                        glow::UNSIGNED_INT,
                        0,
                        matrices.len() as i32,
                    );
                }

                gl.bind_vertex_array(None);
                gl.viewport(
                    saved_viewport[0],
                    saved_viewport[1],
                    saved_viewport[2],
                    saved_viewport[3],
                );
            });

            gl.use_program(None);
        }

        let result_texture = unsafe {
            slint::BorrowedOpenGLTextureBuilder::new_gl_2d_rgba_texture(
                self.next_texture.texture.0,
                (self.next_texture.width, self.next_texture.height).into(),
            )
            .build()
        };

        std::mem::swap(&mut self.next_texture, &mut self.displayed_texture);
        result_texture
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

impl Drop for Renderer {
    fn drop(&mut self) {
        unsafe {
            self.gl.delete_program(self.program);
        }
    }
}
