use bevy_ecs::prelude::*;
use std::error::Error;
use std::ffi::OsStr;

use crate::{
    material::Material, material::MaterialDesc, material_resource::MaterialResource,
    mesh_resource::MeshResource, render_body_resource::RenderBodyResource,
    shader_resource::ShaderResource, texture_resource_manager::TextureResource,
};

#[derive(Resource)]
pub struct RenderResourceManager {
    pub mesh_manager: MeshResource,
    pub render_body_manager: RenderBodyResource,
    pub material_manager: MaterialResource,
    pub texture_manager: TextureResource,
    pub shader_manager: ShaderResource,
}

impl RenderResourceManager {
    pub fn new() -> Self {
        Self {
            mesh_manager: MeshResource::default(),
            render_body_manager: RenderBodyResource::default(),
            material_manager: MaterialResource::default(),
            texture_manager: TextureResource::default(),
            shader_manager: ShaderResource::default(),
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
        let shader_handle = self
            .shader_manager
            .get_or_load(gl, vertex_shader, fragment_shader);
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

            let desc = MaterialDesc::new(
                shader_handle,
                albedo_handle,
                normal_handle,
                roughness,
                base_reflectance,
            );
            let handle = self.material_manager.add_material(Material::new(desc));
            material_handles.push(handle);
        }

        Ok(material_handles)
    }
}
