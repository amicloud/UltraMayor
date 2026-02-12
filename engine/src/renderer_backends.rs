use std::{collections::HashMap, rc::Rc};

use glam::{Mat4, Vec3};
use glow::{Context as GlowContext, HasContext};

use crate::{MaterialHandle, MeshHandle, render_instance::RenderInstance, renderer::VaoKey};

pub trait GraphicsBackend {
    type Buffer;
    type Program;
    type VertexArray;

    // fn create_buffer(&mut self) -> Self::Buffer;

    // fn bind_array_buffer(&mut self, buffer: Option<Self::Buffer>);
    // fn upload_buffer_data(&mut self, data: &[u8], dynamic: bool);
    // fn upload_buffer_sub_data(&mut self, offset: i32, data: &[u8]);

    // fn use_program(&mut self, program: Option<Self::Program>);
    // fn bind_vertex_array(&mut self, vao: Option<Self::VertexArray>);

    // fn draw_elements_instanced(&mut self, count: i32, instance_count: i32);

    fn enable() {}
    fn depth_func() {}
    fn cull_face() {}
    fn clear() {}
    fn viewport() {}
    fn get_parameter_i32_slice() {}
    fn use_program() {}
    fn uniform_1_f32() {}
    fn uniform_3_f32() {}
    fn uniform_1_i32() {}
    fn uniform_matrix_4_f32_slice() {}
    fn create_vertex_array() {}
    fn bind_vertex_array() {}
    fn create_buffer() {}
    fn bind_buffer() {}
    fn buffer_data_u8_slice() {}
    fn buffer_data_size() {}
    fn delete_buffer() {}
    fn enable_vertex_attrib_array() {}
    fn vertex_attrib_pointer_f32() {}
    fn vertex_attrib_divisor() {}
    fn active_texture() {}
    fn bind_texture() {}
    fn draw_elements_instanced() {}
}
pub struct NullBackend {
    pub frames_rendered: u64,
}

pub struct GlowBackend {
    pub gl: Rc<GlowContext>,
    pub frames_rendered: u64,
}

impl GraphicsBackend for NullBackend {
    type Buffer = ();
    type Program = ();
    type VertexArray = ();
}

impl GraphicsBackend for GlowBackend {
    type Buffer = glow::Buffer;
    type Program = glow::Program;
    type VertexArray = glow::VertexArray;
}
