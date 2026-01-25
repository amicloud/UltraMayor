// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.
use approx::relative_eq;
use bytemuck::{Pod, Zeroable};
use glow::HasContext;
use nalgebra::Vector3;
use obj::{load_obj, Obj};
use std::ffi::OsStr;
use std::fs::File;
use std::io::BufReader;
use std::{hash::Hash, hash::Hasher};

#[repr(C)]
#[derive(Default, Clone, Pod, Copy, Debug)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub barycentric: [f32; 3],
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
    #[allow(dead_code)]
    pub fn new(position: [f32; 3], normal: [f32; 3], barycentric: [f32; 3]) -> Self {
        Self {
            position,
            normal,
            barycentric,
        }
    }

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

    #[allow(dead_code)]
    pub fn get_position_vector3(&self) -> Vector3<f32> {
        Vector3::new(self.position[0], self.position[1], self.position[2])
    }
}

#[allow(dead_code)]
#[derive(Default, Clone)]
pub struct AABB {
    min: Vector3<f32>,
    max: Vector3<f32>,
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

    #[allow(dead_code)]
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
    pub id: u32,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,

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
            let vao = gl.create_vertex_array().unwrap();
            let vbo = gl.create_buffer().unwrap();
            let ebo = gl.create_buffer().unwrap();

            let instance_vbo = gl.create_buffer().expect("create instance VBO");

            gl.bind_vertex_array(Some(vao));

            // Vertex buffer
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            gl.buffer_data_u8_slice(
                glow::ARRAY_BUFFER,
                bytemuck::cast_slice(&self.vertices),
                glow::STATIC_DRAW,
            );

            // Index buffer
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ebo));
            gl.buffer_data_u8_slice(
                glow::ELEMENT_ARRAY_BUFFER,
                bytemuck::cast_slice(&self.indices),
                glow::STATIC_DRAW,
            );

            let vertex_stride: i32 = std::mem::size_of::<Vertex>() as i32;
            let position_size = 3;
            let normal_size = 3;
            let barycentric_size = 3;

            // Offsets
            let position_offset = 0;
            let normal_offset = position_offset + position_size;
            let barycentric_offset = normal_offset + normal_size;

            // Enable attributes
            gl.enable_vertex_attrib_array(0); // position
            gl.vertex_attrib_pointer_f32(
                0,
                position_size,
                glow::FLOAT,
                false,
                vertex_stride,
                position_offset * 4,
            );

            gl.enable_vertex_attrib_array(1); // normal
            gl.vertex_attrib_pointer_f32(
                1,
                normal_size,
                glow::FLOAT,
                true,
                vertex_stride,
                normal_offset * 4,
            );

            gl.enable_vertex_attrib_array(2); // barycentric
            gl.vertex_attrib_pointer_f32(
                2,
                barycentric_size,
                glow::FLOAT,
                true,
                vertex_stride,
                barycentric_offset * 4,
            );

            // Instance buffer: reserve no data yet (we'll upload per-frame when we know instance count)
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(instance_vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, &[], glow::DYNAMIC_DRAW);

            // The model matrix is 4 vec4 attributes at locations 3..6.
            // Stride = 4 * vec4 = 64 bytes, offsets = 0, 16, 32, 48
            let mat_stride = (16 * std::mem::size_of::<f32>()) as i32; // 64 bytes
            for i in 0..4 {
                let loc = 3 + i; // 3,4,5,6
                gl.enable_vertex_attrib_array(loc);
                gl.vertex_attrib_pointer_f32(
                    loc,
                    4,
                    glow::FLOAT,
                    false,
                    mat_stride,
                    (i * 16) as i32, // offset in bytes (i * vec4 size = i * 16 bytes? careful)
                );
                // Important: the offset argument of vertex_attrib_pointer_f32 expects bytes.
                // (i * 16) is wrong if we think in bytes; correct is (i * 16) bytes? Actually each vec4 is 4 floats -> 16 bytes.
                // So use i * 16 (bytes), but because vertex_attrib_pointer_f32 expects i32 offset in BYTES, we compute:
                // (i * 16) // 16 bytes per vec4
                // Here we used (i * 16) already (it is bytes).
                gl.vertex_attrib_divisor(loc, 1); // this makes it per-instance
            }

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

    /// Upload per-instance model matrices (slice of `[f32;16]`) to the instance VBO.
    pub fn update_instance_buffer(&mut self, instance_matrices: &[[f32; 16]], gl: &glow::Context) {
        if instance_matrices.is_empty() {
            self.instance_count = 0;
            return;
        }
        unsafe {
            let instance_buf = self.instance_vbo.expect("no instance buffer");
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(instance_buf));
            gl.buffer_data_u8_slice(
                glow::ARRAY_BUFFER,
                bytemuck::cast_slice(instance_matrices),
                glow::DYNAMIC_DRAW,
            );
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            self.instance_count = instance_matrices.len();
        }
    }

    pub fn from_obj(path: &OsStr) -> Result<Self, Box<dyn std::error::Error>> {
        let input = BufReader::new(File::open(path)?);
        let obj: Obj = load_obj(input)?;
        let mut mesh = Mesh::default();

        #[inline]
        pub fn f32x3_to_f32x3(v: [f32; 3]) -> [f32; 3] {
            [v[0] as f32, v[1] as f32, v[2] as f32]
        }

        mesh.vertices = obj
            .vertices
            .into_iter()
            .map(|pos| Vertex {
                position: f32x3_to_f32x3(pos.position),
                normal: f32x3_to_f32x3(pos.normal),
                barycentric: [0.0, 0.0, 0.0],
            })
            .collect();
        mesh.indices = obj.indices.iter().map(|&i| i as u32).collect();
        mesh.id = {
            let mut hasher = std::collections::hash_map::DefaultHasher::new();
            path.hash(&mut hasher);
            hasher.finish() as u32
        };
        Ok(mesh)
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
