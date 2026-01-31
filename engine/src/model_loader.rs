use log::warn;
use std::collections::hash_map::DefaultHasher;
use std::ffi::OsStr;
use std::hash::{Hash, Hasher};

use crate::handles::{MaterialHandle, MeshHandle, RenderBodyHandle};
use crate::mesh::Mesh;
use crate::render_body::{RenderBody, RenderBodyPart};
use crate::render_resource_manager::RenderResourceManager;
use crate::Engine;

impl Engine {
    /// Loads a model from the specified file path. Supports different model formats based on file extension.
    /// Returns a `RenderBodyHandle` if the model is successfully loaded, or `None` if the format is unsupported.
    ///
    /// Currently supported formats: glTF (.gltf)
    ///
    /// FBX (.fbx) loading is not yet implemented.
    pub fn load_model(&mut self, model_path: &str) -> Option<RenderBodyHandle> {
        // Get the file extension to determine the loader
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
        let base_dir = obj_path.parent().unwrap_or_else(|| std::path::Path::new("."));

        let (models, materials) = tobj::load_obj(
            obj_path,
            &tobj::LoadOptions {
                single_index: true,
                ..Default::default()
            },
        )
        .expect("Failed to load OBJ file");

        let render_body_handle = {
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

            if let Ok(materials) = materials.as_ref() {
                for material in materials {
                    let albedo_handle = if let Some(tex) = material.diffuse_texture.as_ref() {
                        if !tex.is_empty() {
                            let tex_path = base_dir.join(tex);
                            render_resource_manager
                                .texture_manager
                                .load_from_file(gl, tex_path.as_os_str())
                        } else {
                            let diffuse = material.diffuse.unwrap_or([1.0, 1.0, 1.0]);
                            let rgba = [
                                (diffuse[0].clamp(0.0, 1.0) * 255.0) as u8,
                                (diffuse[1].clamp(0.0, 1.0) * 255.0) as u8,
                                (diffuse[2].clamp(0.0, 1.0) * 255.0) as u8,
                                255,
                            ];
                            render_resource_manager.texture_manager.create_solid_rgba(gl, rgba)
                        }
                    } else {
                        let diffuse = material.diffuse.unwrap_or([1.0, 1.0, 1.0]);
                        let rgba = [
                            (diffuse[0].clamp(0.0, 1.0) * 255.0) as u8,
                            (diffuse[1].clamp(0.0, 1.0) * 255.0) as u8,
                            (diffuse[2].clamp(0.0, 1.0) * 255.0) as u8,
                            255,
                        ];
                        render_resource_manager.texture_manager.create_solid_rgba(gl, rgba)
                    };

                    let normal_handle = if let Some(tex) = material.normal_texture.as_ref() {
                        if !tex.is_empty() {
                            let tex_path = base_dir.join(tex);
                            Some(
                                render_resource_manager
                                    .texture_manager
                                    .load_from_file(gl, tex_path.as_os_str()),
                            )
                        } else {
                            None
                        }
                    } else {
                        None
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

                    let desc = crate::material::MaterialDesc::new(
                        shader_handle,
                        albedo_handle,
                        normal_handle,
                        roughness,
                        0.04,
                    );
                    let handle = render_resource_manager
                        .material_manager
                        .add_material(crate::material::Material::new(desc));
                    material_handles.push(handle);
                }
            }

            let default_material = if !material_handles.is_empty() {
                material_handles[0]
            } else {
                let albedo = render_resource_manager
                    .texture_manager
                    .create_solid_rgba(gl, [255, 255, 255, 255]);
                let desc = crate::material::MaterialDesc::new(
                    shader_handle,
                    albedo,
                    None,
                    1.0,
                    0.04,
                );
                render_resource_manager
                    .material_manager
                    .add_material(crate::material::Material::new(desc))
            };

            let mut parts = Vec::with_capacity(models.len());
            for (model_index, model) in models.iter().enumerate() {
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

                let indices: Vec<u32> = mesh.indices.iter().copied().collect();

                let tangents = if has_uvs && has_normals {
                    Mesh::compute_tangents(&positions, &normals, &uvs, &indices)
                } else {
                    vec![[1.0, 0.0, 0.0, 1.0]; vertex_count]
                };

                let mut built_mesh = Mesh::default();
                for i in 0..vertex_count {
                    built_mesh.vertices.push(crate::mesh::Vertex {
                        position: positions[i],
                        normal: normals[i],
                        barycentric: [0.0, 0.0, 0.0],
                        uv_albedo: uvs[i],
                        uv_normal: uvs[i],
                        tangent: tangents[i],
                    });
                }
                built_mesh.indices.extend(indices.iter().copied());

                built_mesh.id = {
                    let mut hasher = DefaultHasher::new();
                    obj_path.hash(&mut hasher);
                    model_index.hash(&mut hasher);
                    MeshHandle(hasher.finish() as u32)
                };
                built_mesh.aabb = crate::mesh::AABB::from_vertices(&built_mesh.vertices);
                built_mesh.compute_bounding_sphere();

                let mesh_handle = render_resource_manager.mesh_manager.add_mesh(built_mesh, gl);
                let material_handle = mesh
                    .material_id
                    .and_then(|idx| material_handles.get(idx).copied())
                    .unwrap_or(default_material);

                parts.push(RenderBodyPart {
                    mesh_id: mesh_handle,
                    material_id: material_handle,
                    local_transform: nalgebra::Matrix4::identity(),
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
        };

        render_body_handle
    }

    /// Loads an FBX model from the specified file path and returns a `RenderBodyHandle`.
    fn load_fbx(&mut self, _fbx_path: &str) -> RenderBodyHandle {
        unimplemented!("FBX loading is not yet implemented");
    }

    // Not particularly happy with how this works, but it will do for now.
    // It's a bit of a disorganized mess
    /// Loads a glTF model from the specified file path and returns a `RenderBodyHandle`.
    fn load_gltf(&mut self, gltf_path: &str) -> RenderBodyHandle {
        let gl = &self.gl;
        let os_path = OsStr::new(gltf_path);
        let render_body_handle = {
            let mut render_resource_manager = self
                .world
                .get_resource_mut::<RenderResourceManager>()
                .expect("RenderResourceManager resource not found");

            let mesh_primitives = Mesh::from_gltf(os_path).unwrap();

            let material_handles = render_resource_manager
                .load_materials_from_gltf(
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
                let mesh_handle = render_resource_manager.mesh_manager.add_mesh(prim.mesh, gl);

                let material_handle = prim
                    .material_index
                    .and_then(|idx| material_handles.get(idx).copied())
                    .unwrap_or(default_material);

                parts.push(RenderBodyPart {
                    mesh_id: mesh_handle,
                    material_id: material_handle,
                    local_transform: nalgebra::Matrix4::identity(),
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
        };

        render_body_handle
    }
}
