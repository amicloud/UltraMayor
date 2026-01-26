use crate::handles::TextureHandle;
use glow::HasContext;

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

    /// Upload raw RGBA bytes to GPU
    pub fn upload_to_gpu(&mut self, gl: &glow::Context, data: &[u8]) {
        unsafe {
            let tex = gl.create_texture().expect("Failed to create texture");
            gl.bind_texture(glow::TEXTURE_2D, Some(tex));
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MIN_FILTER,
                glow::LINEAR as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MAG_FILTER,
                glow::LINEAR as i32,
            );
            gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_WRAP_S, glow::REPEAT as i32);
            gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_WRAP_T, glow::REPEAT as i32);

            gl.tex_image_2d(
                glow::TEXTURE_2D,
                0,
                glow::RGBA as i32,
                self.width as i32,
                self.height as i32,
                0,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                Some(data),
            );

            gl.bind_texture(glow::TEXTURE_2D, None);
            self.gl_tex = Some(tex);
        }
    }
}
