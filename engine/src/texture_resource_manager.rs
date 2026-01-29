use crate::handles::TextureHandle;
use crate::texture::Texture;
use glow::Context;
use image::GenericImageView;
use std::collections::HashMap;
use std::error::Error;
use std::ffi::OsStr;
use std::hash::{Hash, Hasher}; 

#[derive(Default)]
pub struct TextureResource {
    pub textures: HashMap<TextureHandle, Texture>,
    pub default_normal_map: TextureHandle,
}

#[allow(dead_code)]
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

    fn gltf_image_to_rgba(image: &gltf::image::Data) -> Result<Vec<u8>, String> {
        use gltf::image::Format;

        let rgba = match image.format {
            Format::R8 => image
                .pixels
                .iter()
                .flat_map(|r| [*r, *r, *r, 255])
                .collect(),
            Format::R8G8 => image
                .pixels
                .chunks(2)
                .flat_map(|rg| [rg[0], rg[1], 0, 255])
                .collect(),
            Format::R8G8B8 => image
                .pixels
                .chunks(3)
                .flat_map(|rgb| [rgb[0], rgb[1], rgb[2], 255])
                .collect(),
            Format::R8G8B8A8 => image.pixels.clone(),
            _ => return Err(format!("Unsupported glTF image format: {:?}", image.format)),
        };

        Ok(rgba)
    }

    #[allow(dead_code)]
    pub fn load_from_file(&mut self, gl: &Context, path: &OsStr) -> TextureHandle {
        // Load image with the `image` crate
        let img = image::open(path).expect("Failed to open texture image");
        let rgba = img.to_rgba8();
        let (width, height) = img.dimensions();
        let id = self.make_hashed_id(&path);

        let mut tex = Texture::new(id, width, height);
        tex.upload_to_gpu(gl, &rgba);
        self.add_texture(tex)
    }

    pub fn create_solid_rgba(&mut self, gl: &Context, rgba: [u8; 4]) -> TextureHandle {
        let id = self.make_hashed_id(&("solid_rgba", rgba));
        let mut tex = Texture::new(id, 1, 1);
        tex.upload_to_gpu(gl, &rgba);
        self.add_texture(tex)
    }

    #[allow(dead_code)]
    pub fn load_from_gltf(
        &mut self,
        gl: &Context,
        path: &OsStr,
    ) -> Result<HashMap<usize, TextureHandle>, Box<dyn Error>> {
        let path_str = path.to_str().ok_or("Invalid UTF-8 in glTF path")?;
        let (gltf, _buffers, images) = gltf::import(path_str)?;
        self.load_from_gltf_data(gl, path, &gltf, &images)
    }

    pub fn load_from_gltf_data(
        &mut self,
        gl: &Context,
        path: &OsStr,
        gltf: &gltf::Document,
        images: &[gltf::image::Data],
    ) -> Result<HashMap<usize, TextureHandle>, Box<dyn Error>> {
        let mut texture_map = HashMap::new();

        for texture in gltf.textures() {
            let texture_index = texture.index();
            let image_index = texture.source().index();
            let image = images
                .get(image_index)
                .ok_or("glTF image index out of bounds")?;
            let rgba = Self::gltf_image_to_rgba(image)
                .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;

            let id = self.make_hashed_id(&(path, texture_index));
            let mut tex = Texture::new(id, image.width, image.height);
            tex.upload_to_gpu(gl, &rgba);
            let handle = self.add_texture(tex);
            texture_map.insert(texture_index, handle);
        }

        Ok(texture_map)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn make_image_data(format: gltf::image::Format, pixels: Vec<u8>) -> gltf::image::Data {
        gltf::image::Data {
            pixels,
            width: 1,
            height: 1,
            format,
        }
    }

    #[test]
    fn test_gltf_image_to_rgba_r8() {
        let image = make_image_data(gltf::image::Format::R8, vec![10]);
        let rgba = TextureResource::gltf_image_to_rgba(&image).unwrap();
        assert_eq!(rgba, vec![10, 10, 10, 255]);
    }

    #[test]
    fn test_gltf_image_to_rgba_r8g8() {
        let image = make_image_data(gltf::image::Format::R8G8, vec![10, 20]);
        let rgba = TextureResource::gltf_image_to_rgba(&image).unwrap();
        assert_eq!(rgba, vec![10, 20, 0, 255]);
    }

    #[test]
    fn test_gltf_image_to_rgba_r8g8b8() {
        let image = make_image_data(gltf::image::Format::R8G8B8, vec![10, 20, 30]);
        let rgba = TextureResource::gltf_image_to_rgba(&image).unwrap();
        assert_eq!(rgba, vec![10, 20, 30, 255]);
    }

    #[test]
    fn test_gltf_image_to_rgba_r8g8b8a8() {
        let image = make_image_data(gltf::image::Format::R8G8B8A8, vec![10, 20, 30, 40]);
        let rgba = TextureResource::gltf_image_to_rgba(&image).unwrap();
        assert_eq!(rgba, vec![10, 20, 30, 40]);
    }

    #[test]
    fn test_gltf_image_to_rgba_unsupported_format() {
        let image = make_image_data(gltf::image::Format::R16, vec![0, 0]);
        let result = TextureResource::gltf_image_to_rgba(&image);
        assert!(result.is_err());
    }
}
