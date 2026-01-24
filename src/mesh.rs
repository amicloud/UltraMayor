// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.
use approx::relative_eq;
use bytemuck::{Pod, Zeroable};
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

#[derive(Default, Clone)]
pub struct Mesh {
    pub id: u32,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

impl Mesh {
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
