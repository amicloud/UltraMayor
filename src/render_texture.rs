// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.
use std::rc::Rc;
slint::include_modules!();
use crate::ScopedFrameBufferBinding;
use glow::HasContext;

pub struct RenderTexture {
    pub texture: glow::Texture,
    pub depth_texture: glow::Texture,
    pub width: u32,
    pub height: u32,
    pub fbo: glow::Framebuffer,
    pub gl: Rc<glow::Context>,
}

impl RenderTexture {
    pub unsafe fn new(gl: &Rc<glow::Context>, width: u32, height: u32) -> Self {
        // Create framebuffer
        let fbo = gl
            .create_framebuffer()
            .expect("Unable to create framebuffer");
        let _saved_fbo_binding = ScopedFrameBufferBinding::new(gl, Some(fbo));

        // Create color texture
        let texture = gl.create_texture().expect("Unable to allocate texture");
        gl.bind_texture(glow::TEXTURE_2D, Some(texture));
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
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_WRAP_S,
            glow::CLAMP_TO_EDGE as i32,
        );
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_WRAP_T,
            glow::CLAMP_TO_EDGE as i32,
        );
        gl.tex_image_2d(
            glow::TEXTURE_2D,
            0,
            glow::RGBA as i32,
            width as i32,
            height as i32,
            0,
            glow::RGBA,
            glow::UNSIGNED_BYTE,
            None,
        );

        // Attach color texture to framebuffer
        gl.framebuffer_texture_2d(
            glow::FRAMEBUFFER,
            glow::COLOR_ATTACHMENT0,
            glow::TEXTURE_2D,
            Some(texture),
            0,
        );

        // Create depth texture
        let depth_texture = gl.create_texture().expect("Unable to create depth texture");
        gl.bind_texture(glow::TEXTURE_2D, Some(depth_texture));
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_MIN_FILTER,
            glow::NEAREST as i32,
        );
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_MAG_FILTER,
            glow::NEAREST as i32,
        );
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_WRAP_S,
            glow::CLAMP_TO_EDGE as i32,
        );
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_WRAP_T,
            glow::CLAMP_TO_EDGE as i32,
        );
        gl.tex_image_2d(
            glow::TEXTURE_2D,
            0,
            glow::DEPTH_COMPONENT24 as i32,
            width as i32,
            height as i32,
            0,
            glow::DEPTH_COMPONENT,
            glow::UNSIGNED_INT,
            None,
        );

        // Attach depth texture to framebuffer
        gl.framebuffer_texture_2d(
            glow::FRAMEBUFFER,
            glow::DEPTH_ATTACHMENT,
            glow::TEXTURE_2D,
            Some(depth_texture),
            0,
        );

        // Ensure the framebuffer is complete
        let status = gl.check_framebuffer_status(glow::FRAMEBUFFER);
        if status != glow::FRAMEBUFFER_COMPLETE {
            println!("Framebuffer is not complete! Status: {:?}", status);
            panic!("Framebuffer is not complete!");
        }

        Self {
            texture,
            depth_texture,
            width,
            height,
            fbo,
            gl: gl.clone(),
        }
    }

    pub unsafe fn with_texture_as_active_fbo<R>(&self, callback: impl FnOnce() -> R) -> R {
        let _saved_fbo = ScopedFrameBufferBinding::new(&self.gl, Some(self.fbo));
        callback()
    }
}

impl Drop for RenderTexture {
    fn drop(&mut self) {
        unsafe {
            self.gl.delete_framebuffer(self.fbo);
            self.gl.delete_texture(self.texture);
            self.gl.delete_texture(self.depth_texture);
        }
    }
}
