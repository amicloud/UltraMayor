use glow::Context;
use image::GenericImageView;
use slotmap::SlotMap;
use std::ffi::OsStr;

use crate::assets::{handles::TextureHandle, texture::Texture};
use crate::render::renderer;

#[derive(Default)]
pub struct TextureResource {
    pub textures: SlotMap<TextureHandle, Texture>,
    pub default_normal_map: TextureHandle,
}

impl TextureResource {
    pub fn add_texture(&mut self, texture: Texture) -> TextureHandle {
        self.textures.insert(texture)
    }

    pub fn load_from_file(&mut self, gl: &Context, path: &OsStr) -> TextureHandle {
        // Load image with the `image` crate
        let img = image::open(path)
            .unwrap_or_else(|_| panic!("Failed to open texture image: {:?}", path));
        let rgba = img.to_rgba8();
        let (width, height) = img.dimensions();

        let mut tex = Texture::new(width, height);
        renderer::Renderer::upload_texture_to_gpu(&mut tex, gl, &rgba);
        self.add_texture(tex)
    }

    pub fn create_solid_rgba(&mut self, gl: &Context, rgba: [u8; 4]) -> TextureHandle {
        let mut tex = Texture::new(1, 1);
        renderer::Renderer::upload_texture_to_gpu(&mut tex, gl, &rgba);
        self.add_texture(tex)
    }

    pub(crate) fn create_from_rgba_with_key(
        &mut self,
        gl: &Context,
        width: u32,
        height: u32,
        rgba: &[u8],
    ) -> TextureHandle {
        let mut tex = Texture::new(width, height);
        renderer::Renderer::upload_texture_to_gpu(&mut tex, gl, rgba);
        self.add_texture(tex)
    }

    pub fn create_default_normal_map(&mut self, gl: &Context) -> TextureHandle {
        // 1x1 RGBA8 (128, 128, 255, 255)
        let pixels: [u8; 4] = [128, 128, 255, 255];
        let mut tex = Texture::new(1, 1);

        renderer::Renderer::upload_texture_to_gpu(&mut tex, gl, &pixels);
        let id = self.add_texture(tex);
        self.default_normal_map = id;
        id
    }

    pub fn get_texture(&self, id: TextureHandle) -> Option<&Texture> {
        self.textures.get(id)
    }
}
