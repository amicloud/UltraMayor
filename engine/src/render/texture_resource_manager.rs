use crate::handles::TextureHandle;
use crate::render::renderer;
use crate::render::texture::Texture;
use glow::Context;
use image::GenericImageView;
use std::collections::HashMap;
use std::ffi::OsStr;
use std::hash::{Hash, Hasher};

#[derive(Default)]
pub struct TextureResource {
    pub textures: HashMap<TextureHandle, Texture>,
    pub default_normal_map: TextureHandle,
}

impl TextureResource {
    pub fn add_texture(&mut self, texture: Texture) -> TextureHandle {
        let id = texture.id;
        self.textures.insert(id, texture);
        id
    }

    fn make_hashed_id<T: Hash>(&self, value: &T) -> TextureHandle {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        value.hash(&mut hasher);
        let mut id = TextureHandle(hasher.finish() as u32);
        // The default normal map will always be texture 0.
        if id == TextureHandle(0) {
            id = TextureHandle(1);
        }
        id
    }

    pub fn load_from_file(&mut self, gl: &Context, path: &OsStr) -> TextureHandle {
        // Load image with the `image` crate
        let img = image::open(path).expect(&format!("Failed to open texture image: {:?}", path));
        let rgba = img.to_rgba8();
        let (width, height) = img.dimensions();
        let id = self.make_hashed_id(&path);

        let mut tex = Texture::new(id, width, height);
        renderer::Renderer::upload_texture_to_gpu(&mut tex, gl, &rgba);
        self.add_texture(tex)
    }

    pub fn create_solid_rgba(&mut self, gl: &Context, rgba: [u8; 4]) -> TextureHandle {
        let id = self.make_hashed_id(&("solid_rgba", rgba));
        let mut tex = Texture::new(id, 1, 1);
        renderer::Renderer::upload_texture_to_gpu(&mut tex, gl, &rgba);
        self.add_texture(tex)
    }

    pub(crate) fn create_from_rgba_with_key<T: Hash>(
        &mut self,
        gl: &Context,
        key: &T,
        width: u32,
        height: u32,
        rgba: &[u8],
    ) -> TextureHandle {
        let id = self.make_hashed_id(key);
        let mut tex = Texture::new(id, width, height);
        renderer::Renderer::upload_texture_to_gpu(&mut tex, gl, rgba);
        self.add_texture(tex)
    }

    pub fn create_default_normal_map(&mut self, gl: &Context) -> TextureHandle {
        // 1x1 RGBA8 (128, 128, 255, 255)
        let pixels: [u8; 4] = [128, 128, 255, 255];
        let mut tex = Texture::new(TextureHandle(0), 1, 1);

        renderer::Renderer::upload_texture_to_gpu(&mut tex, gl, &pixels);
        let id = self.add_texture(tex);
        self.default_normal_map = id;
        id
    }

    pub fn get_texture(&self, id: TextureHandle) -> Option<&Texture> {
        self.textures.get(&id)
    }
}
