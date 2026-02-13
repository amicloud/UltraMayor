
use crate::frustum::Frustum;
use crate::handles::MaterialHandle;
use crate::handles::MeshHandle;
use crate::handles::ShaderHandle;
use crate::mesh::Mesh;
use crate::mesh::Vertex;
use crate::render_instance::RenderInstance;
use crate::render_resource_manager::RenderResourceManager;
use crate::shader::InputRate::PerInstance;
use crate::shader::InputRate::PerVertex;
use crate::shader::UniformValue;
use crate::shader::VertexAttribType;
use glam::{Mat4, Vec3};
use glow::Context as GlowContext;
use glow::HasContext;
use std::collections::HashMap;
use std::mem::offset_of;
use std::ops::Range;
use std::rc::Rc;

pub struct Renderer {
    gl: Rc<GlowContext>,
    frames_rendered: u64,
    vao_cache: HashMap<VaoKey, glow::VertexArray>,
    mesh_render_data: HashMap<MeshHandle, MeshRenderData>,
    frame_data: FrameData,
}

pub struct MeshRenderData {
    // GPU handles
    pub vbo: Option<glow::Buffer>,
    pub ebo: Option<glow::Buffer>,
    pub instance_vbo: Option<glow::Buffer>,
    pub instance_count: usize,
}

#[derive(Default)]
struct FrameData {
    /// Instances copied from the render queue at the start of each frame.
    input_instances: Vec<RenderInstance>,
    visible_instances: Vec<RenderInstance>,
    frame_uniforms: FrameUniforms,
    /// Flat storage for all instance matrices in the frame (reused across frames).
    instance_matrices: Vec<[f32; 16]>,
    /// Ranges into `instance_matrices` for each mesh within a material batch.
    mesh_batch_ranges: Vec<MeshBatchRange>,
    /// Ranges into `mesh_batch_ranges` for each material batch.
    material_batch_ranges: Vec<MaterialBatchRange>,
}

struct MaterialBatchRange {
    material_id: MaterialHandle,
    mesh_batches: Range<usize>,
}

struct MeshBatchRange {
    mesh_id: MeshHandle,
    matrices: Range<usize>,
}

#[derive(Default)]
struct FrameUniforms {
    view_proj: Mat4,
    camera_position: Vec3,
    light_direction: Vec3,
    light_color: Vec3,
}

pub struct RenderParams {
    pub width: u32,
    pub height: u32,
}

#[derive(PartialEq, Hash, Eq)]
pub struct VaoKey {
    mesh: MeshHandle,
    shader: ShaderHandle,
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

    /// Copies instances into the renderer's internal buffer.
    /// Call this before `render()` to stage the frame's instances.
    pub fn stage_instances(&mut self, instances: &[RenderInstance]) {
        self.frame_data.input_instances.clear();
        self.frame_data.input_instances.extend_from_slice(instances);
    }

    pub fn render(
        &mut self,
        render_params: RenderParams,
        render_data_manager: &mut RenderResourceManager,
        camera: Option<CameraRenderData>,
    ) {
        let gl = self.gl.clone();
        let current_time = std::time::Instant::now();
        let mut draw_calls = 0;

        unsafe {
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LEQUAL);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
            gl.enable(glow::CULL_FACE);
            gl.cull_face(glow::BACK);
        }

        let saved_viewport = {
            let mut vp = [0; 4];
            unsafe {
                gl.get_parameter_i32_slice(glow::VIEWPORT, &mut vp);
            }
            vp
        };

        unsafe {
            gl.viewport(
                0,
                0,
                render_params.width as i32,
                render_params.height as i32,
            );
        }

        let Some(camera) = camera else {
            unsafe {
                gl.viewport(
                    saved_viewport[0],
                    saved_viewport[1],
                    saved_viewport[2],
                    saved_viewport[3],
                );
                return;
            }
        };

        let view_proj = camera.view_proj;

        // Set common uniforms
        let default_light_color = Vec3::new(1.0, 1.0, 1.0);

        self.frame_data.frame_uniforms.view_proj = view_proj;
        self.frame_data.frame_uniforms.camera_position = camera.position;
        self.frame_data.frame_uniforms.light_direction = Vec3::new(0.0, 0.0, 1.0);
        self.frame_data.frame_uniforms.light_color = default_light_color;

        Self::frustum_culling(
            &mut self.frame_data.visible_instances,
            &self.frame_data.input_instances,
            render_data_manager,
            &view_proj,
        );

        Self::material_batcher(
            &mut self.frame_data.visible_instances,
            &mut self.frame_data.material_batch_ranges,
            &mut self.frame_data.mesh_batch_ranges,
            &mut self.frame_data.instance_matrices,
        );

        for mat_idx in 0..self.frame_data.material_batch_ranges.len() {
            let material_id = self.frame_data.material_batch_ranges[mat_idx].material_id;
            let mesh_range = self.frame_data.material_batch_ranges[mat_idx]
                .mesh_batches
                .clone();

            let material = render_data_manager
                .material_manager
                .get_material(material_id)
                .expect("Material not found");
            let shader = render_data_manager
                .shader_manager
                .get_shader(material.desc.shader)
                .expect("Shader not found");

            // Bind shader
            unsafe {
                gl.use_program(Some(shader.program));
            }

            // 1. Bind frame uniforms
            if let Some(loc) = shader.uniforms.get("u_view_proj") {
                Self::bind_uniform(
                    &gl,
                    loc,
                    &UniformValue::Mat4(self.frame_data.frame_uniforms.view_proj),
                    render_data_manager,
                );
            }
            if let Some(loc) = shader.uniforms.get("u_camera_position") {
                Self::bind_uniform(
                    &gl,
                    loc,
                    &UniformValue::Vec3(self.frame_data.frame_uniforms.camera_position),
                    render_data_manager,
                );
            }
            if let Some(loc) = shader.uniforms.get("u_light_direction") {
                Self::bind_uniform(
                    &gl,
                    loc,
                    &UniformValue::Vec3(self.frame_data.frame_uniforms.light_direction),
                    render_data_manager,
                );
            }
            if let Some(loc) = shader.uniforms.get("u_light_color") {
                Self::bind_uniform(
                    &gl,
                    loc,
                    &UniformValue::Vec3(self.frame_data.frame_uniforms.light_color),
                    render_data_manager,
                );
            }

            let mut textures_bound: u32 = 0;

            // 2. Bind material uniforms
            for (name, value) in &material.desc.params {
                if let Some(loc) = shader.uniforms.get(name) {
                    textures_bound += Self::bind_uniform(&gl, loc, value, render_data_manager);
                }
            }

            // Draw each mesh
            for mesh_idx in mesh_range {
                let mesh_id = self.frame_data.mesh_batch_ranges[mesh_idx].mesh_id;
                let matrices_range = self.frame_data.mesh_batch_ranges[mesh_idx].matrices.clone();
                let matrices_slice = &self.frame_data.instance_matrices[matrices_range];

                let vao = Self::get_or_create_vao(
                    &mut self.vao_cache,
                    &gl,
                    &mesh_id,
                    &material.desc.shader,
                    render_data_manager,
                    &mut self.mesh_render_data,
                );
                Self::update_instance_buffer(
                    &gl,
                    mesh_id,
                    &mut self.mesh_render_data,
                    matrices_slice,
                );

                let index_count: i32 = {
                    render_data_manager
                        .mesh_manager
                        .get_mesh(mesh_id)
                        .expect(&format!("Couldn't find mesh: {:?}", mesh_id))
                        .indices
                        .len() as i32
                };

                unsafe {
                    gl.bind_vertex_array(Some(vao));
                    gl.draw_elements_instanced(
                        glow::TRIANGLES,
                        index_count,
                        glow::UNSIGNED_INT,
                        0,
                        matrices_slice.len() as i32,
                    );
                }
                draw_calls += 1;
            }

            // Now that we are done with the material, unbind any textures it used
            for unit in 0..textures_bound {
                unsafe {
                    gl.active_texture(glow::TEXTURE0 + unit);
                    gl.bind_texture(glow::TEXTURE_2D, None);
                }
            }
        }

        unsafe {
            gl.bind_vertex_array(None);
            gl.viewport(
                saved_viewport[0],
                saved_viewport[1],
                saved_viewport[2],
                saved_viewport[3],
            );
        }

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

    /// Groups visible instances into material → mesh batches using a sort
    /// instead of hash maps. All output is written into caller-owned `Vec`s
    /// that are `.clear()`-ed here and reused across frames, so after the
    /// first few frames there are zero allocations.
    fn material_batcher(
        instances: &mut Vec<RenderInstance>,
        material_ranges: &mut Vec<MaterialBatchRange>,
        mesh_ranges: &mut Vec<MeshBatchRange>,
        matrices: &mut Vec<[f32; 16]>,
    ) {
        material_ranges.clear();
        mesh_ranges.clear();
        matrices.clear();

        // Sort by (material, mesh) so identical keys are contiguous.
        instances.sort_unstable_by(|a, b| {
            a.material_id
                .cmp(&b.material_id)
                .then(a.mesh_id.cmp(&b.mesh_id))
        });

        let mut i = 0;
        while i < instances.len() {
            let material_id = instances[i].material_id;
            let mesh_batches_start = mesh_ranges.len();

            // Walk all instances that share this material.
            while i < instances.len() && instances[i].material_id == material_id {
                let mesh_id = instances[i].mesh_id;
                let matrices_start = matrices.len();

                // Walk all instances that share this material AND mesh.
                while i < instances.len()
                    && instances[i].material_id == material_id
                    && instances[i].mesh_id == mesh_id
                {
                    matrices.push(instances[i].transform.to_cols_array());
                    i += 1;
                }

                mesh_ranges.push(MeshBatchRange {
                    mesh_id,
                    matrices: matrices_start..matrices.len(),
                });
            }

            material_ranges.push(MaterialBatchRange {
                material_id,
                mesh_batches: mesh_batches_start..mesh_ranges.len(),
            });
        }
    }

    pub fn frustum_culling(
        visible_instances: &mut Vec<RenderInstance>,
        instances: &[RenderInstance],
        render_data_manager: &RenderResourceManager,
        view_proj: &Mat4,
    ) {
        visible_instances.clear();
        let frustum = Frustum::from_view_proj(view_proj);

        for inst in instances {
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
                visible_instances.push(inst.clone());
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
                vao_cache: HashMap::new(),
                frame_data: FrameData::default(),
                mesh_render_data: HashMap::new(),
            };
            renderer
        }
    }

    pub fn upload_to_gpu(
        gl: &glow::Context,
        mesh: &Mesh,
        mesh_render_data: &mut HashMap<MeshHandle, MeshRenderData>,
    ) {
        unsafe {
            // Unbind any active VAO so that EBO binding below does not
            // corrupt a previously-created VAO's element-buffer state.
            gl.bind_vertex_array(None);

            // --- Create GPU objects ---
            let vbo = gl.create_buffer().unwrap();
            let ebo = gl.create_buffer().unwrap();
            let instance_vbo = gl.create_buffer().unwrap();

            // Upload vertex data
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            gl.buffer_data_u8_slice(
                glow::ARRAY_BUFFER,
                bytemuck::cast_slice(&mesh.vertices),
                glow::STATIC_DRAW,
            );

            // Upload index data
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ebo));
            gl.buffer_data_u8_slice(
                glow::ELEMENT_ARRAY_BUFFER,
                bytemuck::cast_slice(&mesh.indices),
                glow::STATIC_DRAW,
            );

            // Allocate empty instance buffer (resized on update)
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(instance_vbo));
            gl.buffer_data_size(glow::ARRAY_BUFFER, 0, glow::DYNAMIC_DRAW);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, None);

            let mesh_data = MeshRenderData {
                vbo: Some(vbo),
                ebo: Some(ebo),
                instance_vbo: Some(instance_vbo),
                instance_count: 0,
            };
            mesh_render_data.insert(mesh.id, mesh_data);
        }
    }

    pub fn update_instance_buffer(
        gl: &glow::Context,
        mesh_handle: MeshHandle,
        mesh_render_data: &mut HashMap<MeshHandle, MeshRenderData>,
        instance_matrices: &[[f32; 16]],
    ) {
        let mesh_data = mesh_render_data
            .get_mut(&mesh_handle)
            .expect("Mesh not found");
        if instance_matrices.is_empty() {
            mesh_data.instance_count = 0;
            return;
        }

        let instance_buf = mesh_data.instance_vbo.unwrap();

        unsafe {
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(instance_buf));
            gl.buffer_data_u8_slice(
                glow::ARRAY_BUFFER,
                bytemuck::cast_slice(instance_matrices),
                glow::DYNAMIC_DRAW,
            );
            gl.bind_buffer(glow::ARRAY_BUFFER, None);

            mesh_data.instance_count = instance_matrices.len();
        }
    }

    fn get_or_create_vao(
        vao_cache: &mut HashMap<VaoKey, glow::VertexArray>,
        gl: &glow::Context,
        mesh: &MeshHandle,
        shader: &ShaderHandle,
        render_data_manager: &RenderResourceManager,
        mesh_render_data: &mut HashMap<MeshHandle, MeshRenderData>,
    ) -> glow::VertexArray {
        let key = VaoKey {
            mesh: *mesh,
            shader: *shader,
        };

        if let Some(vao) = vao_cache.get(&key) {
            return *vao;
        }

        if !mesh_render_data.contains_key(mesh) {
            let mesh_data = render_data_manager
                .mesh_manager
                .get_mesh(*mesh)
                .expect("Mesh not found");
            Self::upload_to_gpu(gl, mesh_data, mesh_render_data);
        }

        let mesh_data = mesh_render_data.get(mesh).unwrap();
        let vertex_stride = crate::mesh::Vertex::stride();

        unsafe {
            let vao = gl.create_vertex_array().unwrap();
            gl.bind_vertex_array(Some(vao));

            gl.bind_buffer(glow::ARRAY_BUFFER, Some(mesh_data.vbo.unwrap()));
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(mesh_data.ebo.unwrap()));

            let instance_offset_for = |name: &str| -> Option<i32> {
                if let Some(suffix) = name.strip_prefix("instance_model_col") {
                    if let Ok(index) = suffix.parse::<i32>() {
                        if (0..4).contains(&index) {
                            return Some(index * 16);
                        }
                    }
                }
                None
            };

            for attrib in &render_data_manager
                .shader_manager
                .get_shader(*shader)
                .unwrap()
                .attributes
            {
                let location = attrib.location;
                let divisor = match attrib.rate {
                    PerVertex => 0,
                    PerInstance => 1,
                };

                match attrib.rate {
                    PerInstance => {
                        if let Some(offset) = instance_offset_for(&attrib.name) {
                            gl.bind_buffer(
                                glow::ARRAY_BUFFER,
                                Some(mesh_data.instance_vbo.unwrap()),
                            );
                            gl.enable_vertex_attrib_array(location);
                            gl.vertex_attrib_pointer_f32(
                                location,
                                4,
                                glow::FLOAT,
                                false,
                                64,
                                offset,
                            );
                            gl.vertex_attrib_divisor(location, divisor);
                        }
                    }
                    PerVertex => {
                        if let Some(offset) = {
                            let name: &str = &attrib.name;
                            match name {
                                "position" => Some(offset_of!(Vertex, position) as i32),
                                "normal" => Some(offset_of!(Vertex, normal) as i32),
                                "barycentric" => Some(offset_of!(Vertex, barycentric) as i32),
                                "uv_albedo" => Some(offset_of!(Vertex, uv_albedo) as i32),
                                "uv_normal" => Some(offset_of!(Vertex, uv_normal) as i32),
                                "tangent" => Some(offset_of!(Vertex, tangent) as i32),
                                _ => None,
                            }
                        } {
                            gl.bind_buffer(glow::ARRAY_BUFFER, Some(mesh_data.vbo.unwrap()));
                            gl.enable_vertex_attrib_array(location);
                            let (size, ty) = match attrib.ty {
                                VertexAttribType::Float32 => (1, glow::FLOAT),
                                VertexAttribType::Vec2 => (2, glow::FLOAT),
                                VertexAttribType::Vec3 => (3, glow::FLOAT),
                                VertexAttribType::Vec4 => (4, glow::FLOAT),
                            };
                            gl.vertex_attrib_pointer_f32(
                                location,
                                size,
                                ty,
                                false,
                                vertex_stride,
                                offset,
                            );
                            gl.vertex_attrib_divisor(location, divisor);
                        }
                    }
                }
            }

            gl.bind_vertex_array(None);
            vao_cache.insert(key, vao);
            vao
        }
    }

    /// Deletes a mesh's GPU resources
    #[allow(dead_code)]
    pub fn delete_mesh_gpu(&mut self, mesh_handle: MeshHandle) {
        let mesh_data = self
            .mesh_render_data
            .get_mut(&mesh_handle)
            .expect("Mesh not found");
        unsafe {
            if let Some(vbo) = mesh_data.vbo.take() {
                self.gl.delete_buffer(vbo);
            }
            if let Some(ebo) = mesh_data.ebo.take() {
                self.gl.delete_buffer(ebo);
            }
            if let Some(inst) = mesh_data.instance_vbo.take() {
                self.gl.delete_buffer(inst);
            }
            mesh_data.instance_count = 0;
        }
    }

    /// This returns an int to indicate how many texture units were bound
    /// (so they can be unbound later). Not sure if this is clever or gross.
    fn bind_uniform(
        gl: &glow::Context,
        loc: &glow::UniformLocation,
        value: &UniformValue,
        render_data_manager: &RenderResourceManager,
    ) -> u32 {
        unsafe {
            match value {
                UniformValue::Float(v) => {
                    gl.uniform_1_f32(Some(loc), *v);
                    return 0;
                }
                UniformValue::Vec3(v) => {
                    gl.uniform_3_f32(Some(loc), v.x, v.y, v.z);
                    return 0;
                }
                UniformValue::Mat4(m) => {
                    gl.uniform_matrix_4_f32_slice(Some(loc), false, &m.to_cols_array());
                    return 0;
                }
                UniformValue::Int(i) => {
                    gl.uniform_1_i32(Some(loc), *i);
                    return 0;
                }
                UniformValue::Texture { handle, unit } => {
                    let tex = render_data_manager
                        .texture_manager
                        .get_texture(*handle)
                        .expect("Texture missing");

                    gl.active_texture(glow::TEXTURE0 + *unit);
                    gl.bind_texture(glow::TEXTURE_2D, tex.gl_tex);
                    gl.uniform_1_i32(Some(loc), *unit as i32);
                    return 1;
                }
            }
        }
    }
}
