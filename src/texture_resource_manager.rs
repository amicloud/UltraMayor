use crate::handles::TextureHandle;
use crate::texture::Texture;
use glow::Context;
use image::GenericImageView;
use std::collections::HashMap;
use std::ffi::OsStr; // cargo add image

#[derive(Default)]
pub struct TextureResourceManager {
    pub textures: HashMap<TextureHandle, Texture>,
}

impl TextureResourceManager {
    pub fn add_texture(&mut self, texture: Texture) -> TextureHandle {
        let id = texture.id;
        self.textures.insert(id, texture);
        id
    }

    pub fn load_from_file(
        &mut self,
        gl: &Context,
        path: &OsStr,
        id: TextureHandle,
    ) -> TextureHandle {
        // Load image with the `image` crate
        let img = image::open(path)
            .expect("Failed to open texture image")
            .flipv();
        let rgba = img.to_rgba8();
        let (width, height) = img.dimensions();

        let mut tex = Texture::new(id, width, height);
        tex.upload_to_gpu(gl, &rgba);

        let id = self.add_texture(tex);
        id
    }

    pub fn get_texture(&self, id: TextureHandle) -> Option<&Texture> {
        self.textures.get(&id)
    }
}
