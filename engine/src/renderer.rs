use crate::handles::MaterialHandle;
use crate::handles::MeshHandle;
use crate::mesh::Mesh;
use crate::render_instance::RenderInstance;
use crate::render_resource_manager::RenderResourceManager;
use crate::shader::UniformValue;
use std::rc::Rc;
// use crate::render_texture::RenderTexture;
use glam::{Mat4, Vec3};
use glow::Context as GlowContext;
use glow::HasContext;
pub struct Renderer {
    gl: Rc<GlowContext>,
    // displayed_texture: RenderTexture,
    // next_texture: RenderTexture,
    frames_rendered: u64,
}
pub struct RenderParams {
    pub width: u32,
    pub height: u32,
}

/// Precomputed camera data required by the renderer.
pub struct CameraRenderData {
    pub view_proj: Mat4,
    pub position: Vec3,
}
impl Renderer {
    fn max_scale(mat: Mat4) -> f32 {
        let x = mat.x_axis.truncate().length();
        let y = mat.y_axis.truncate().length();
        let z = mat.z_axis.truncate().length();
        x.max(y).max(z)
    }

    pub fn render(
        &mut self,
        render_params: RenderParams,
        render_data_manager: &mut RenderResourceManager,
        instances: Vec<RenderInstance>,
        camera: Option<CameraRenderData>,
    ) {
        unsafe {
            let gl = &self.gl;
            let current_time = std::time::Instant::now();
            let mut draw_calls = 0;

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
                render_params.width as i32,
                render_params.height as i32,
            );

            let Some(camera) = camera else {
                gl.viewport(
                    saved_viewport[0],
                    saved_viewport[1],
                    saved_viewport[2],
                    saved_viewport[3],
                );
                return;
            };

            let view_proj = camera.view_proj;

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

                let scale = Self::max_scale(inst.transform);
                let world_center = inst
                    .transform
                    .transform_point3(Vec3::from(mesh.sphere_center));
                let world_radius = mesh.sphere_radius * scale;

                if frustum.intersects_sphere(world_center, world_radius) {
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

            // Set common uniforms
            let default_light_color = Vec3::new(1.0, 1.0, 1.0);
            let mut frame_uniforms = HashMap::<String, UniformValue>::new();
            frame_uniforms.insert("u_view_proj".into(), UniformValue::Mat4(view_proj));
            frame_uniforms.insert(
                "u_camera_position".into(),
                UniformValue::Vec3(camera.position),
            );
            frame_uniforms.insert(
                "u_light_direction".into(),
                UniformValue::Vec3(Vec3::new(0.0, 0.0, 1.0)),
            );
            frame_uniforms.insert(
                "u_light_color".into(),
                UniformValue::Vec3(default_light_color),
            );

            // ------------------------------------------------------------
            // PASS 3: Render
            // ------------------------------------------------------------
            for (material_id, inst_group) in instances_by_material {
                let material = render_data_manager
                    .material_manager
                    .get_material(material_id)
                    .expect("Material not found");
                let shader = render_data_manager
                    .shader_manager
                    .get_shader(material.desc.shader)
                    .expect("Shader not found");

                // Bind shader
                gl.use_program(Some(shader.program));
                for name in material.desc.params.keys() {
                    if !shader.uniforms.contains_key(name) {
                        println!(
                            "Warning: material provides uniform '{}' not used by shader",
                            name
                        );
                    }
                }

                // 1. Bind frame uniforms
                for (name, value) in &frame_uniforms {
                    if let Some(loc) = shader.uniforms.get(name) {
                        Self::bind_uniform(gl, loc, value, render_data_manager);
                    }
                }

                // 2. Bind material uniforms
                for (name, value) in &material.desc.params {
                    if let Some(loc) = shader.uniforms.get(name) {
                        Self::bind_uniform(gl, loc, value, render_data_manager);
                    }
                }

                // Group by mesh
                let mut instances_by_mesh: HashMap<MeshHandle, Vec<[f32; 16]>> = HashMap::new();
                for inst in inst_group {
                    instances_by_mesh
                        .entry(inst.mesh_id)
                        .or_default()
                        .push(inst.transform.to_cols_array());
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
                // Now that we are done with the material, unbind any textures it used
                for unit in 0..16 {
                    gl.active_texture(glow::TEXTURE0 + unit);
                    gl.bind_texture(glow::TEXTURE_2D, None);
                }
            }

            gl.bind_vertex_array(None);
            gl.viewport(
                saved_viewport[0],
                saved_viewport[1],
                saved_viewport[2],
                saved_viewport[3],
            );

            self.frames_rendered += 1;

            if self.frames_rendered % 60 == 0 {
                println!("Frames rendered: {}", self.frames_rendered);
                println!(
                    "Render time: {:.2} ms",
                    current_time.elapsed().as_secs_f32() * 1000.0
                );
                println!("Draw calls on last frame: {}", draw_calls);
            }
        }
    }

    pub fn new(gl: Rc<GlowContext>) -> Self {
        unsafe {
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);

            let renderer = Self {
                gl,
                frames_rendered: 0,
            };
            renderer
        }
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

    fn bind_uniform(
        gl: &glow::Context,
        loc: &glow::UniformLocation,
        value: &UniformValue,
        render_data_manager: &RenderResourceManager,
    ) {
        unsafe {
            match value {
                UniformValue::Float(v) => {
                    gl.uniform_1_f32(Some(loc), *v);
                }
                UniformValue::Vec3(v) => {
                    gl.uniform_3_f32(Some(loc), v.x, v.y, v.z);
                }
                UniformValue::Mat4(m) => {
                    gl.uniform_matrix_4_f32_slice(Some(loc), false, &m.to_cols_array());
                }
                UniformValue::Int(i) => {
                    gl.uniform_1_i32(Some(loc), *i);
                }
                UniformValue::Texture { handle, unit } => {
                    let tex = render_data_manager
                        .texture_manager
                        .get_texture(*handle)
        .unwrap_or_else(|| render_data_manager.texture_manager.get_texture(render_data_manager.texture_manager.default_normal_map).unwrap()); 

                    gl.active_texture(glow::TEXTURE0 + *unit);
                    gl.bind_texture(glow::TEXTURE_2D, tex.gl_tex);
                    gl.uniform_1_i32(Some(loc), *unit as i32);
                }
            }
        }
    }
}
