use log::warn;
use std::{
    collections::{HashMap, hash_map::DefaultHasher},
    ffi::OsStr,
    hash::{Hash, Hasher},
};

use crate::{
    Engine,
    assets::{
        handles::{MaterialHandle, RenderBodyHandle, ShaderHandle, TextureHandle},
        material::{Material, MaterialDesc},
        mesh::{Aabb, GltfPrimitiveMesh, Mesh, Vertex},
        shader::UniformValue,
        texture_resource_manager::TextureResource,
    },
    render::{
        render_body::{RenderBody, RenderBodyPart},
        render_resource_manager::RenderResourceManager,
    },
};

impl Engine {
    fn rgba_from_rgb(rgb: [f32; 3]) -> [u8; 4] {
        [
            (rgb[0].clamp(0.0, 1.0) * 255.0) as u8,
            (rgb[1].clamp(0.0, 1.0) * 255.0) as u8,
            (rgb[2].clamp(0.0, 1.0) * 255.0) as u8,
            255,
        ]
    }

    fn rgba_from_rgba_f32(rgba: [f32; 4]) -> [u8; 4] {
        [
            (rgba[0].clamp(0.0, 1.0) * 255.0) as u8,
            (rgba[1].clamp(0.0, 1.0) * 255.0) as u8,
            (rgba[2].clamp(0.0, 1.0) * 255.0) as u8,
            (rgba[3].clamp(0.0, 1.0) * 255.0) as u8,
        ]
    }

    fn create_pbr_material(
        render_resource_manager: &mut RenderResourceManager,
        shader_handle: ShaderHandle,
        albedo_handle: TextureHandle,
        normal_handle: TextureHandle,
        roughness: f32,
    ) -> MaterialHandle {
        let mut params = Vec::new();
        params.push(("u_roughness".to_string(), UniformValue::Float(roughness)));
        params.push(("u_base_reflectance".to_string(), UniformValue::Float(0.04)));
        params.push((
            "u_albedo".to_string(),
            UniformValue::Texture {
                handle: albedo_handle,
                unit: 0,
            },
        ));
        params.push((
            "u_normal".to_string(),
            UniformValue::Texture {
                handle: normal_handle,
                unit: 1,
            },
        ));

        let desc = MaterialDesc::new(shader_handle, params);
        render_resource_manager
            .material_manager
            .add_material(Material::new(desc))
    }

    /// Loads a model from the specified file path. Supports different model formats based on file extension.
    /// Returns a `RenderBodyHandle` if the model is successfully loaded, or `None` if the format is unsupported.
    ///
    /// Currently supported formats: glTF (.gltf)
    ///
    /// FBX (.fbx) loading is not yet implemented.
    pub fn load_model(&mut self, model_path: &str) -> Option<RenderBodyHandle> {
        let extension = std::path::Path::new(model_path)
            .extension()
            .and_then(|ext| ext.to_str())
            .unwrap_or("")
            .to_lowercase();

        match extension.as_str() {
            "gltf" | "glb" => Some(self.load_gltf(model_path)),
            "fbx" => Some(self.load_fbx(model_path)),
            "obj" => Some(self.load_obj(model_path)),
            _ => {
                warn!("Unsupported model format: {}", extension);
                None
            }
        }
    }

    /// Loads an OBJ model from the specified file path and returns a `RenderBodyHandle`.
    fn load_obj(&mut self, obj_path: &str) -> RenderBodyHandle {
        let gl = &self.gl;
        let obj_path = std::path::Path::new(obj_path);
        let base_dir = obj_path
            .parent()
            .unwrap_or_else(|| std::path::Path::new("."));

        let (models, obj_materials) = tobj::load_obj(
            obj_path,
            &tobj::LoadOptions {
                single_index: true,
                ..Default::default()
            },
        )
        .expect("Failed to load OBJ file");

        {
            let mut render_resource_manager = self
                .world
                .get_resource_mut::<RenderResourceManager>()
                .expect("RenderResourceManager resource not found");

            let shader_handle = render_resource_manager.shader_manager.get_or_load(
                gl,
                OsStr::new("resources/shaders/pbr.vert"),
                OsStr::new("resources/shaders/pbr.frag"),
            );

            let mut material_handles: Vec<MaterialHandle> = Vec::new();

            if let Ok(obj_materials) = obj_materials.as_ref() {
                for material in obj_materials {
                    let albedo_handle = if let Some(tex) = material.diffuse_texture.as_ref() {
                        if !tex.is_empty() {
                            let tex_path = base_dir.join(tex);
                            render_resource_manager
                                .texture_manager
                                .load_from_file(gl, tex_path.as_os_str())
                        } else {
                            let diffuse = material.diffuse.unwrap_or([1.0, 1.0, 1.0]);
                            let rgba = Self::rgba_from_rgb(diffuse);
                            render_resource_manager
                                .texture_manager
                                .create_solid_rgba(gl, rgba)
                        }
                    } else {
                        let diffuse = material.diffuse.unwrap_or([1.0, 1.0, 1.0]);
                        let rgba = Self::rgba_from_rgb(diffuse);
                        render_resource_manager
                            .texture_manager
                            .create_solid_rgba(gl, rgba)
                    };

                    let normal_handle = if let Some(tex) = material.normal_texture.as_ref() {
                        if !tex.is_empty() {
                            let tex_path = base_dir.join(tex);
                            render_resource_manager
                                .texture_manager
                                .load_from_file(gl, tex_path.as_os_str())
                        } else {
                            render_resource_manager.texture_manager.default_normal_map
                        }
                    } else {
                        render_resource_manager.texture_manager.default_normal_map
                    };

                    let roughness = if let Some(shininess) = material.shininess {
                        if shininess > 0.0 {
                            (1.0_f32 - (shininess / 1000.0)).clamp(0.0, 1.0)
                        } else {
                            1.0
                        }
                    } else {
                        1.0
                    };

                    let handle = Self::create_pbr_material(
                        &mut render_resource_manager,
                        shader_handle,
                        albedo_handle,
                        normal_handle,
                        roughness,
                    );
                    material_handles.push(handle);
                }
            }

            let default_material = if !material_handles.is_empty() {
                material_handles[0]
            } else {
                let albedo = render_resource_manager
                    .texture_manager
                    .create_solid_rgba(gl, [255, 255, 255, 255]);
                let default_normal = render_resource_manager.texture_manager.default_normal_map;
                Self::create_pbr_material(
                    &mut render_resource_manager,
                    shader_handle,
                    albedo,
                    default_normal,
                    1.0,
                )
            };

            let mut parts = Vec::with_capacity(models.len());
            for (_model_index, model) in models.iter().enumerate() {
                let mesh = &model.mesh;
                let vertex_count = mesh.positions.len() / 3;

                let mut positions: Vec<[f32; 3]> = Vec::with_capacity(vertex_count);
                for i in 0..vertex_count {
                    positions.push([
                        mesh.positions[i * 3],
                        mesh.positions[i * 3 + 1],
                        mesh.positions[i * 3 + 2],
                    ]);
                }

                let has_normals = !mesh.normals.is_empty();
                let mut normals: Vec<[f32; 3]> = Vec::with_capacity(vertex_count);
                if has_normals {
                    for i in 0..vertex_count {
                        normals.push([
                            mesh.normals[i * 3],
                            mesh.normals[i * 3 + 1],
                            mesh.normals[i * 3 + 2],
                        ]);
                    }
                } else {
                    normals.resize(vertex_count, [0.0, 0.0, 1.0]);
                }

                let has_uvs = !mesh.texcoords.is_empty();
                let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(vertex_count);
                if has_uvs {
                    for i in 0..vertex_count {
                        let u = mesh.texcoords[i * 2];
                        let v = mesh.texcoords[i * 2 + 1];
                        uvs.push([u, 1.0 - v]);
                    }
                } else {
                    uvs.resize(vertex_count, [0.0, 0.0]);
                }

                let indices: Vec<u32> = mesh.indices.to_vec();

                let tangents = if has_uvs && has_normals {
                    Mesh::compute_tangents(&positions, &normals, &uvs, &indices)
                } else {
                    vec![[1.0, 0.0, 0.0, 1.0]; vertex_count]
                };

                let mut built_mesh = Mesh::default();
                for i in 0..vertex_count {
                    built_mesh.vertices.push(Vertex {
                        position: positions[i],
                        normal: normals[i],
                        barycentric: [0.0, 0.0, 0.0],
                        uv_albedo: uvs[i],
                        uv_normal: uvs[i],
                        tangent: tangents[i],
                    });
                }
                built_mesh.indices.extend(indices.iter().copied());

                built_mesh.aabb = Aabb::from_vertices(&built_mesh.vertices);
                built_mesh.compute_bounding_sphere();
                built_mesh.build_bvh(8);

                let mesh_handle = render_resource_manager.mesh_manager.add_mesh(built_mesh);
                let material_handle = mesh
                    .material_id
                    .and_then(|idx| material_handles.get(idx).copied())
                    .unwrap_or(default_material);

                parts.push(RenderBodyPart {
                    mesh_id: mesh_handle,
                    material_id: material_handle,
                    local_transform: glam::Mat4::IDENTITY,
                });
            }

            let render_body_id = {
                let mut hasher = DefaultHasher::new();
                obj_path.hash(&mut hasher);
                RenderBodyHandle(hasher.finish() as u32)
            };

            let render_body = RenderBody::new(render_body_id, parts);
            render_resource_manager
                .render_body_manager
                .add_render_body(render_body)
        }
    }

    /// Loads an FBX model from the specified file path and returns a `RenderBodyHandle`.
    fn load_fbx(&mut self, _fbx_path: &str) -> RenderBodyHandle {
        unimplemented!("FBX loading is not yet implemented");
    }

    /// Loads a glTF model from the specified file path and returns a `RenderBodyHandle`.
    fn load_gltf(&mut self, gltf_path: &str) -> RenderBodyHandle {
        let gl = &self.gl;
        let os_path = OsStr::new(gltf_path);

        {
            let mut render_resource_manager = self
                .world
                .get_resource_mut::<RenderResourceManager>()
                .expect("RenderResourceManager resource not found");

            let mesh_primitives = Self::mesh_primatives_from_gltf(os_path).unwrap();

            let material_handles = Self::load_materials_from_gltf(
                &mut render_resource_manager,
                gl,
                os_path,
                OsStr::new("resources/shaders/pbr.vert"),
                OsStr::new("resources/shaders/pbr.frag"),
            )
            .expect("Failed to load glTF materials");

            let default_material = *material_handles
                .first()
                .expect("No materials found in glTF");

            let mut parts = Vec::with_capacity(mesh_primitives.len());
            for prim in mesh_primitives {
                let mesh_handle = render_resource_manager.mesh_manager.add_mesh(prim.mesh);

                let material_handle = prim
                    .material_index
                    .and_then(|idx| material_handles.get(idx).copied())
                    .unwrap_or(default_material);

                parts.push(RenderBodyPart {
                    mesh_id: mesh_handle,
                    material_id: material_handle,
                    local_transform: glam::Mat4::IDENTITY,
                });
            }

            let render_body_id = {
                let mut hasher = DefaultHasher::new();
                gltf_path.hash(&mut hasher);
                RenderBodyHandle(hasher.finish() as u32)
            };

            let render_body = RenderBody::new(render_body_id, parts);
            render_resource_manager
                .render_body_manager
                .add_render_body(render_body)
        }
    }

    fn load_materials_from_gltf(
        render_resource_manager: &mut RenderResourceManager,
        gl: &glow::Context,
        gltf_path: &OsStr,
        vertex_shader: &OsStr,
        fragment_shader: &OsStr,
    ) -> Result<Vec<MaterialHandle>, Box<dyn std::error::Error>> {
        let path_str = gltf_path.to_str().ok_or("Invalid UTF-8 in glTF path")?;
        let (gltf, _buffers, images) = gltf::import(path_str)?;

        let texture_map = Self::load_textures_from_gltf_data(
            &mut render_resource_manager.texture_manager,
            gl,
            gltf_path,
            &gltf,
            &images,
        )?;

        let mut material_handles = Vec::new();
        let shader_handle =
            render_resource_manager
                .shader_manager
                .get_or_load(gl, vertex_shader, fragment_shader);
        for material in gltf.materials() {
            let pbr = material.pbr_metallic_roughness();
            let roughness = pbr.roughness_factor();

            let albedo_handle = pbr
                .base_color_texture()
                .and_then(|info| texture_map.get(&info.texture().index()).copied())
                .unwrap_or_else(|| {
                    let base_color = pbr.base_color_factor();
                    let rgba = Self::rgba_from_rgba_f32(base_color);
                    render_resource_manager
                        .texture_manager
                        .create_solid_rgba(gl, rgba)
                });

            let normal_handle = material
                .normal_texture()
                .and_then(|info| texture_map.get(&info.texture().index()).copied())
                .unwrap_or(render_resource_manager.texture_manager.default_normal_map);

            let handle = Self::create_pbr_material(
                render_resource_manager,
                shader_handle,
                albedo_handle,
                normal_handle,
                roughness,
            );
            material_handles.push(handle);
        }

        Ok(material_handles)
    }

    fn load_textures_from_gltf_data(
        texture_manager: &mut TextureResource,
        gl: &glow::Context,
        path: &OsStr,
        gltf: &gltf::Document,
        images: &[gltf::image::Data],
    ) -> Result<HashMap<usize, TextureHandle>, Box<dyn std::error::Error>> {
        let mut texture_map = HashMap::new();

        for texture in gltf.textures() {
            let texture_index = texture.index();
            let image_index = texture.source().index();
            let image = images
                .get(image_index)
                .ok_or("glTF image index out of bounds")?;
            let rgba = Self::gltf_image_to_rgba(image)
                .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;

            let handle = texture_manager.create_from_rgba_with_key(
                gl,
                &(path, texture_index),
                image.width,
                image.height,
                &rgba,
            );
            texture_map.insert(texture_index, handle);
        }

        Ok(texture_map)
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

    fn mesh_primatives_from_gltf(
        path: &OsStr,
    ) -> Result<Vec<GltfPrimitiveMesh>, Box<dyn std::error::Error>> {
        let (gltf, buffers, _) = gltf::import(path.to_str().unwrap())?;
        let mut meshes = Vec::new();

        for gltf_mesh in gltf.meshes() {
            println!("Mesh #{}", gltf_mesh.index());

            for primitive in gltf_mesh.primitives() {
                println!("- Primitive #{}", primitive.index());

                let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));

                // Mandatory attributes
                let positions: Vec<[f32; 3]> = reader
                    .read_positions()
                    .ok_or("Mesh missing positions")?
                    .collect();

                let normals: Vec<[f32; 3]> = reader
                    .read_normals()
                    .ok_or("Mesh missing normals")?
                    .collect();

                let uvs: Vec<[f32; 2]> = reader
                    .read_tex_coords(0)
                    .ok_or("Mesh missing TEXCOORD_0")?
                    .into_f32()
                    // .map(|[u, v]| [u, 1.0 - v])
                    .collect();

                // Indices (required for tangent generation)
                let indices: Vec<u32> = reader
                    .read_indices()
                    .ok_or("Mesh missing indices")?
                    .into_u32()
                    .collect();

                // Some meshes might not have tangents
                // If so, we need to compute them

                let tangents: Vec<[f32; 4]> = if let Some(tangent_reader) = reader.read_tangents() {
                    tangent_reader.collect()
                } else {
                    warn!("Mesh is missing tangents, computing tangents.");
                    Mesh::compute_tangents(&positions, &normals, &uvs, &indices)
                };

                // Sanity check
                assert_eq!(positions.len(), normals.len());
                assert_eq!(positions.len(), uvs.len());
                assert_eq!(positions.len(), tangents.len());

                let mut mesh = Mesh::default();

                // Build vertices
                for i in 0..positions.len() {
                    mesh.vertices.push(Vertex {
                        position: positions[i],
                        normal: normals[i],
                        barycentric: [0.0, 0.0, 0.0],
                        uv_albedo: uvs[i],
                        uv_normal: uvs[i],
                        tangent: [
                            tangents[i][0],
                            tangents[i][1],
                            tangents[i][2],
                            tangents[i][3],
                        ],
                    });
                }

                mesh.indices.extend(indices);
                mesh.aabb = Aabb::from_vertices(&mesh.vertices);
                mesh.compute_bounding_sphere();
                mesh.build_bvh(8);

                meshes.push(GltfPrimitiveMesh {
                    mesh,
                    material_index: primitive.material().index(),
                });
            }
        }

        Ok(meshes)
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
    fn gltf_image_to_rgba_r8() {
        let image = make_image_data(gltf::image::Format::R8, vec![10]);
        let rgba = Engine::gltf_image_to_rgba(&image).unwrap();
        assert_eq!(rgba, vec![10, 10, 10, 255]);
    }

    #[test]
    fn gltf_image_to_rgba_r8g8() {
        let image = make_image_data(gltf::image::Format::R8G8, vec![10, 20]);
        let rgba = Engine::gltf_image_to_rgba(&image).unwrap();
        assert_eq!(rgba, vec![10, 20, 0, 255]);
    }

    #[test]
    fn gltf_image_to_rgba_r8g8b8() {
        let image = make_image_data(gltf::image::Format::R8G8B8, vec![10, 20, 30]);
        let rgba = Engine::gltf_image_to_rgba(&image).unwrap();
        assert_eq!(rgba, vec![10, 20, 30, 255]);
    }

    #[test]
    fn gltf_image_to_rgba_r8g8b8a8() {
        let image = make_image_data(gltf::image::Format::R8G8B8A8, vec![10, 20, 30, 40]);
        let rgba = Engine::gltf_image_to_rgba(&image).unwrap();
        assert_eq!(rgba, vec![10, 20, 30, 40]);
    }

    #[test]
    fn gltf_image_to_rgba_unsupported_format() {
        let image = make_image_data(gltf::image::Format::R16, vec![0, 0]);
        let result = Engine::gltf_image_to_rgba(&image);
        assert!(result.is_err());
    }
}
