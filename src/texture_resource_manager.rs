use crate::handles::TextureHandle;
use crate::texture::Texture;
use glow::Context;
use image::GenericImageView;
use std::collections::HashMap;
use std::ffi::OsStr;
use std::hash::{Hash, Hasher}; // cargo add image

#[derive(Default)]
pub struct TextureResourceManager {
    pub textures: HashMap<TextureHandle, Texture>,
    pub default_normal_map: TextureHandle,
}

impl TextureResourceManager {
    pub fn add_texture(&mut self, texture: Texture) -> TextureHandle {
        let id = texture.id;
        self.textures.insert(id, texture);
        id
    }

    pub fn load_from_file(&mut self, gl: &Context, path: &OsStr) -> TextureHandle {
        // Load image with the `image` crate
        let img = image::open(path)
            .expect("Failed to open texture image")
            .flipv();
        let rgba = img.to_rgba8();
        let (width, height) = img.dimensions();
        let mut id: TextureHandle = {
            let mut hasher = std::collections::hash_map::DefaultHasher::new();
            path.hash(&mut hasher);
            TextureHandle(hasher.finish() as u32)
        };
        // The default normal map will always be texture 0.
        // Hash collision is almost impossible but if something ever does collide, just set it to 1 for now...
        if id == TextureHandle(0) {
            id = TextureHandle(1);
        }

        let mut tex = Texture::new(id, width, height);
        tex.upload_to_gpu(gl, &rgba);
        self.add_texture(tex)
    }

    pub fn create_default_normal_map(&mut self, gl: &Context) -> TextureHandle {
        // 1x1 RGBA8 (128, 128, 255, 255)
        let pixels: [u8; 4] = [128, 128, 255, 255];
        let mut tex = Texture::new(TextureHandle(0), 1, 1);

        tex.upload_to_gpu(gl, &pixels);
        let id = self.add_texture(tex);
        self.default_normal_map = id;
        id
    }

    pub fn get_texture(&self, id: TextureHandle) -> Option<&Texture> {
        self.textures.get(&id)
    }
}
