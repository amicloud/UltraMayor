// use std::{collections::HashMap, rc::Rc};

// use glam::{Mat4, Vec3};
// use glow::{Context as GlowContext, HasContext};

// use crate::{MaterialHandle, MeshHandle, render_instance::RenderInstance, renderer::VaoKey};

// pub trait GraphicsBackend {
//     type Buffer;
//     type Program;
//     type VertexArray;

// }
// pub struct NullBackend {
//     pub frames_rendered: u64,
// }

// pub struct GlowBackend {
//     pub gl: Rc<GlowContext>,
//     pub frames_rendered: u64,
// }

// impl GraphicsBackend for NullBackend {
//     type Buffer = ();
//     type Program = ();
//     type VertexArray = ();
// }

// impl GraphicsBackend for GlowBackend {
//     type Buffer = glow::Buffer;
//     type Program = glow::Program;
//     type VertexArray = glow::VertexArray;
// }
