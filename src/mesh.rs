// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.
use approx::relative_eq;
use bytemuck::{Pod, Zeroable};
use glow::HasContext;
use log::warn;
use nalgebra::Vector3;
use std::ffi::OsStr;
use std::{hash::Hash, hash::Hasher};

use crate::handles::MeshHandle;

#[repr(C)]
#[derive(Default, Clone, Pod, Copy, Debug)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub barycentric: [f32; 3],
    pub uv_albedo: [f32; 2],
    pub uv_normal: [f32; 2],
    pub tangent: [f32; 4],
}

impl PartialEq for Vertex {
    fn eq(&self, other: &Self) -> bool {
        relative_eq!(self.position[0], other.position[0])
            && relative_eq!(self.position[1], other.position[1])
            && relative_eq!(self.position[2], other.position[2])
    }
}

unsafe impl Zeroable for Vertex {
    fn zeroed() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            normal: [0.0, 0.0, 0.0],
            barycentric: [0.0, 0.0, 0.0],
            uv_albedo: [0.0, 0.0],
            uv_normal: [0.0, 0.0],
            tangent: [0.0, 0.0, 0.0, 0.0],
        }
    }
}
impl Eq for Vertex {}

impl Hash for Vertex {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.position_bits().hash(state);
        self.normal_bits().hash(state);
        self.barycentric_bits().hash(state);
    }
}

impl Vertex {
    fn rounded_bits(f: f32, decimal_places: u32) -> u32 {
        // Calculate the scaling factor based on desired decimal places
        let scale = 10f32.powi(decimal_places as i32);
        // Round the floating-point number to the specified decimal places
        let rounded = (f * scale).round();
        // Convert to integer bits for hashing
        rounded.to_bits()
    } /*  */

    // Helper method to get bit representation of normal
    fn position_bits(&self) -> [u32; 3] {
        self.position.map(|f| Self::rounded_bits(f, 5))
    }

    /// Helper method to get bit representation of normal
    fn normal_bits(&self) -> [u32; 3] {
        self.normal.map(|f| Self::rounded_bits(f, 5))
    }

    /// Helper method to get bit representation of normal
    fn barycentric_bits(&self) -> [u32; 3] {
        self.barycentric.map(|f| Self::rounded_bits(f, 5))
    }
}

#[allow(dead_code)]
#[derive(Default, Clone)]
pub struct AABB {
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
}

#[allow(dead_code)]
impl AABB {
    #[allow(dead_code)]
    fn intersect_ray(&self, ray_origin: Vector3<f32>, ray_dir: Vector3<f32>) -> bool {
        let inv_dir = Vector3::new(1.0 / ray_dir.x, 1.0 / ray_dir.y, 1.0 / ray_dir.z);

        let t1 = (self.min.x - ray_origin.x) * inv_dir.x;
        let t2 = (self.max.x - ray_origin.x) * inv_dir.x;
        let t3 = (self.min.y - ray_origin.y) * inv_dir.y;
        let t4 = (self.max.y - ray_origin.y) * inv_dir.y;
        let t5 = (self.min.z - ray_origin.z) * inv_dir.z;
        let t6 = (self.max.z - ray_origin.z) * inv_dir.z;

        let tmin = t1.min(t2).max(t3.min(t4)).max(t5.min(t6));
        let tmax = t1.max(t2).min(t3.max(t4)).min(t5.max(t6));

        tmax >= tmin.max(0.0)
    }

    fn from_vertices(vertices: &Vec<crate::mesh::Vertex>) -> Self {
        // Initialize min and max with the first vertex
        let mut min = vertices[0];
        let mut max = vertices[0];

        // Iterate over all vertices to find min and max values
        for vertex in vertices.iter() {
            min.position[0] = min.position[0].min(vertex.position[0]);
            min.position[1] = min.position[1].min(vertex.position[1]);
            min.position[2] = min.position[2].min(vertex.position[2]);

            max.position[0] = max.position[0].max(vertex.position[0]);
            max.position[1] = max.position[1].max(vertex.position[1]);
            max.position[2] = max.position[2].max(vertex.position[2]);
        }

        AABB {
            min: min.position.into(),
            max: max.position.into(),
        }
    }
}

#[derive(Default, Clone)]
pub struct Mesh {
    pub id: MeshHandle,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub aabb: AABB,
    // Bounding sphere
    pub sphere_center: Vector3<f32>,
    pub sphere_radius: f32,

    // GPU handles
    pub vao: Option<glow::VertexArray>,
    pub vbo: Option<glow::Buffer>,
    pub ebo: Option<glow::Buffer>,

    // Instancing
    pub instance_vbo: Option<glow::Buffer>,
    pub instance_count: usize,
}

impl Mesh {
    pub fn upload_to_gpu(&mut self, gl: &glow::Context) {
        unsafe {
            // --- Create GPU objects ---
            let vao = gl.create_vertex_array().unwrap();
            let vbo = gl.create_buffer().unwrap();
            let ebo = gl.create_buffer().unwrap();
            let instance_vbo = gl.create_buffer().unwrap();

            gl.bind_vertex_array(Some(vao));

            // --- Vertex buffer ---
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            gl.buffer_data_u8_slice(
                glow::ARRAY_BUFFER,
                bytemuck::cast_slice(&self.vertices),
                glow::STATIC_DRAW,
            );

            // --- Index buffer ---
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ebo));
            gl.buffer_data_u8_slice(
                glow::ELEMENT_ARRAY_BUFFER,
                bytemuck::cast_slice(&self.indices),
                glow::STATIC_DRAW,
            );

            let stride = std::mem::size_of::<Vertex>() as i32;

            let position_offset = 0;
            let normal_offset = 3;
            let bary_offset = normal_offset + 3;
            let uv_albedo_offset = bary_offset + 3;
            let uv_normal_offset = uv_albedo_offset + 2;
            let tangent_offset = uv_normal_offset + 2;

            gl.enable_vertex_attrib_array(0);
            gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, stride, position_offset * 4);

            gl.enable_vertex_attrib_array(1);
            gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, stride, normal_offset * 4);

            gl.enable_vertex_attrib_array(2);
            gl.vertex_attrib_pointer_f32(2, 3, glow::FLOAT, false, stride, bary_offset * 4);

            gl.enable_vertex_attrib_array(3);
            gl.vertex_attrib_pointer_f32(3, 2, glow::FLOAT, false, stride, uv_albedo_offset * 4);

            gl.enable_vertex_attrib_array(4);
            gl.vertex_attrib_pointer_f32(4, 2, glow::FLOAT, false, stride, uv_normal_offset * 4);

            gl.enable_vertex_attrib_array(5);
            gl.vertex_attrib_pointer_f32(5, 4, glow::FLOAT, false, stride, tangent_offset * 4);

            // --- Instance buffer ---
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(instance_vbo));
            let max_instances = 10000;
            gl.buffer_data_size(
                glow::ARRAY_BUFFER,
                (max_instances * 16 * 4) as i32, // 16 floats per matrix * 4 bytes
                glow::DYNAMIC_DRAW,
            );

            let mat_stride = 16 * 4; // bytes
            for i in 0..4 {
                let loc = 6 + i; // attributes 6,7,8,9
                gl.enable_vertex_attrib_array(loc);
                gl.vertex_attrib_pointer_f32(
                    loc,
                    4,
                    glow::FLOAT,
                    false,
                    mat_stride,
                    (i * 4 * 4) as i32,
                );
                gl.vertex_attrib_divisor(loc, 1);
            }

            // --- Unbind ---
            gl.bind_vertex_array(None);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, None);

            self.vao = Some(vao);
            self.vbo = Some(vbo);
            self.ebo = Some(ebo);
            self.instance_vbo = Some(instance_vbo);
            self.instance_count = 0;
        }
    }

    pub fn update_instance_buffer(&mut self, instance_matrices: &[[f32; 16]], gl: &glow::Context) {
        if instance_matrices.is_empty() {
            self.instance_count = 0;
            return;
        }

        unsafe {
            let instance_buf = self.instance_vbo.unwrap();
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(instance_buf));

            let byte_len = (instance_matrices.len() * 16 * 4) as i32;

            let ptr = gl.map_buffer_range(
                glow::ARRAY_BUFFER,
                0,
                byte_len,
                glow::MAP_WRITE_BIT
                    | glow::MAP_INVALIDATE_BUFFER_BIT
                    | glow::MAP_UNSYNCHRONIZED_BIT,
            );

            if ptr.is_null() {
                panic!("Failed to map instance buffer");
            }

            let float_ptr = ptr as *mut f32;

            std::ptr::copy_nonoverlapping(
                instance_matrices.as_ptr() as *const f32,
                float_ptr,
                instance_matrices.len() * 16,
            );

            gl.unmap_buffer(glow::ARRAY_BUFFER);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);

            self.instance_count = instance_matrices.len();
        }
    }

    pub fn from_gltf(path: &OsStr) -> Result<Self, Box<dyn std::error::Error>> {
        let (gltf, buffers, _) = gltf::import(path.to_str().unwrap())?;
        let mut mesh = Mesh::default();

        for gltf_mesh in gltf.meshes() {
            println!("Mesh #{}", gltf_mesh.index());
            if gltf_mesh.index() > 0 {
                warn!("Trying to load a glTF with more than 1 mesh, which is currently not supported.");
            }

            for primitive in gltf_mesh.primitives() {
                println!("- Primitive #{}", primitive.index());
                if primitive.index() > 0 {
                    warn!("Trying to load a glTF with more than 1 primitive, which is currently not supported.");
                }

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

                let tangents: Vec<[f32; 4]> = reader
                    .read_tangents()
                    .ok_or("Mesh missing tangents")?
                    .collect();

                // Sanity check
                assert_eq!(positions.len(), normals.len());
                assert_eq!(positions.len(), uvs.len());
                assert_eq!(positions.len(), tangents.len());

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

                // Read indices
                let indices: Vec<u32> = reader
                    .read_indices()
                    .ok_or("Mesh missing indices")?
                    .into_u32()
                    .collect();
                mesh.indices.extend(indices);
            }
        }

        // Mesh ID & bounds
        mesh.id = {
            let mut hasher = std::collections::hash_map::DefaultHasher::new();
            path.hash(&mut hasher);
            MeshHandle(hasher.finish() as u32)
        };
        mesh.aabb = AABB::from_vertices(&mesh.vertices);
        mesh.compute_bounding_sphere();

        Ok(mesh)
    }

    pub fn compute_bounding_sphere(&mut self) {
        // Center = AABB center
        self.sphere_center = (self.aabb.min + self.aabb.max) * 0.5;
        // Radius = distance from center to farthest corner
        self.sphere_radius = (self.aabb.max - self.sphere_center).norm();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default() {
        let mesh = Mesh::default();

        assert!(mesh.vertices.is_empty(), "Default vertices should be empty");
        assert!(mesh.indices.is_empty(), "Default indices should be empty");
    }
}
