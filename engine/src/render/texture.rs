use crate::handles::TextureHandle;

#[derive(Debug)]
pub struct Texture {
    pub id: TextureHandle,
    pub width: u32,
    pub height: u32,
    pub gl_tex: Option<glow::Texture>, // GPU handle
}

impl Texture {
    pub fn new(id: TextureHandle, width: u32, height: u32) -> Self {
        Self {
            id,
            width,
            height,
            gl_tex: None,
        }
    }
}
