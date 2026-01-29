use bevy_ecs::prelude::*;
use std::error::Error;
use std::ffi::OsStr;

use crate::{
    camera_resource::CameraResource, material::Material, material::MaterialDesc,
    material_resource::MaterialResource, mesh_resource::MeshResource, shader::Shader,
    texture_resource_manager::TextureResource,
};

#[derive(Resource)]
pub struct RenderResourceManager {
    pub mesh_manager: MeshResource,
    pub material_manager: MaterialResource,
    pub texture_manager: TextureResource,
    pub camera_manager: CameraResource,
}

impl RenderResourceManager {
    pub fn new() -> Self {
        Self {
            mesh_manager: MeshResource::default(),
            material_manager: MaterialResource::default(),
            texture_manager: TextureResource::default(),
            camera_manager: CameraResource::default(),
        }
    }

    pub fn load_materials_from_gltf(
        &mut self,
        gl: &glow::Context,
        gltf_path: &OsStr,
        vertex_shader: &OsStr,
        fragment_shader: &OsStr,
    ) -> Result<Vec<crate::handles::MaterialHandle>, Box<dyn Error>> {
        let path_str = gltf_path.to_str().ok_or("Invalid UTF-8 in glTF path")?;
        let (gltf, _buffers, images) = gltf::import(path_str)?;

        let texture_map = self
            .texture_manager
            .load_from_gltf_data(gl, gltf_path, &gltf, &images)?;

        let mut material_handles = Vec::new();
        for material in gltf.materials() {
            let pbr = material.pbr_metallic_roughness();
            let roughness = pbr.roughness_factor();
            let base_reflectance = 0.04;

            let albedo_handle = pbr
                .base_color_texture()
                .and_then(|info| texture_map.get(&info.texture().index()).copied())
                .unwrap_or_else(|| {
                    let base_color = pbr.base_color_factor();
                    let rgba = [
                        (base_color[0] * 255.0) as u8,
                        (base_color[1] * 255.0) as u8,
                        (base_color[2] * 255.0) as u8,
                        (base_color[3] * 255.0) as u8,
                    ];
                    self.texture_manager.create_solid_rgba(gl, rgba)
                });

            let normal_handle = material
                .normal_texture()
                .and_then(|info| texture_map.get(&info.texture().index()).copied());

            let shader = Shader::new(gl, vertex_shader, fragment_shader);
            let desc = MaterialDesc::new(
                shader,
                roughness,
                base_reflectance,
                albedo_handle,
                normal_handle,
            );
            let handle = self.material_manager.add_material(Material::new(desc));
            material_handles.push(handle);
        }

        Ok(material_handles)
    }
}
