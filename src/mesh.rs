// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.
use crate::stl_processor::StlProcessorTrait;
use approx::relative_eq;
use bytemuck::{Pod, Zeroable};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::{collections::HashMap, ffi::OsStr, hash::Hash, hash::Hasher};
use stl_io::Triangle;

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

#[derive(Default, Clone, Copy, Debug)]
pub struct SimpleVertex {
    pub position: [f32; 3],
}

impl From<Vector3<f32>> for SimpleVertex {
    fn from(value: Vector3<f32>) -> Self {
        Self {
            position: [value[0],value[1],value[2]]
        }
    }
}
impl PartialEq for SimpleVertex {
    fn eq(&self, other: &Self) -> bool {
        relative_eq!(self.position[0], other.position[0])
            && relative_eq!(self.position[1], other.position[1])
            && relative_eq!(self.position[2], other.position[2])
    }
}

impl Eq for SimpleVertex {}

impl Hash for SimpleVertex {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.position_bits().hash(state);
    }
}

impl SimpleVertex {
    #[allow(dead_code)]
    pub fn new(position: [f32; 3]) -> Self {
        Self { position }
    }

    pub fn apply_rotation(&self, rotation: UnitQuaternion<f32>) -> SimpleVertex {
                // Apply the rotation to the vector using the quaternion
        // The `transform_vector` method applies the rotation without scaling
        let r = rotation.to_rotation_matrix() * self.get_position_vector3();
        SimpleVertex{
            position: [r.x,r.y,r.z]
        }
    }

    fn rounded_bits(f: f32, decimal_places: u32) -> u32 {
        // Calculate the scaling factor based on desired decimal places
        let scale = 10f32.powi(decimal_places as i32);
        // Round the floating-point number to the specified decimal places
        let rounded = (f * scale).round();
        // Convert to integer bits for hashing
        rounded.to_bits()
    }

    // Helper method to get bit representation of normal
    fn position_bits(&self) -> [u32; 3] {
        self.position.map(|f| Self::rounded_bits(f, 5))
    }

    #[allow(dead_code)]
    pub fn get_position_vector3(&self) -> Vector3<f32> {
        Vector3::new(self.position[0], self.position[1], self.position[2])
    }
}

#[derive(Default, Clone)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub simple_vertices: Vec<SimpleVertex>,
    pub simple_indices: Vec<u32>,
}

impl Mesh {
    pub fn get_triangles_for_slicing(&mut self) -> Vec<Triangle> {
        self.into_triangle_vec()
    }

    fn generate_simple_vertices_and_indices(&mut self, original_triangles: &Vec<Triangle>) {
        let mut unique_simple_vertices: Vec<SimpleVertex> = Vec::new();
        let mut simple_indices: Vec<u32> = Vec::new();
        let mut simple_vertex_map: HashMap<SimpleVertex, u32> = HashMap::new();

        for triangle in original_triangles {
            for &vertex_pos in &triangle.vertices {
                let simple_vertex = SimpleVertex {
                    position: vertex_pos,
                };
                let index = if let Some(&existing_index) = simple_vertex_map.get(&simple_vertex) {
                    existing_index
                } else {
                    let new_index = unique_simple_vertices.len() as u32;
                    unique_simple_vertices.push(simple_vertex);
                    simple_vertex_map.insert(simple_vertex, new_index);
                    new_index
                };
                simple_indices.push(index);
            }
        }
        self.simple_vertices = unique_simple_vertices;
        self.simple_indices = simple_indices;
    }

    fn generate_vertices_and_indices(&mut self, original_triangles: &Vec<Triangle>) {
        let mut unique_vertices = Vec::new();
        let mut indices: Vec<u32> = Vec::new();
        let mut vertex_map: HashMap<Vertex, u32> = HashMap::new();

        let b0 = [1.0, 0.0, 0.0];
        let b1 = [0.0, 1.0, 0.0];
        let b2 = [0.0, 0.0, 1.0];

        for triangle in original_triangles {
            let mut bary_num = 0;
            for &vertex_pos in &triangle.vertices {
                let vertex = Vertex {
                    position: vertex_pos,
                    normal: triangle.normal,
                    barycentric: if bary_num == 0 {
                        b0
                    } else if bary_num == 1 {
                        b1
                    } else if bary_num == 2 {
                        b2
                    } else {
                        panic!("Bad barycentric number wtf?");
                    },
                };

                // Insert the vertex into the map if it's not already present
                let index = if let Some(&existing_index) = vertex_map.get(&vertex) {
                    existing_index
                } else {
                    let new_index = unique_vertices.len() as u32;
                    unique_vertices.push(vertex);
                    vertex_map.insert(vertex, new_index);
                    new_index
                };
                indices.push(index);
                bary_num += 1;
            }
        }

        self.vertices = unique_vertices;
        self.indices = indices;
    }

    fn into_triangle_vec(&self) -> Vec<Triangle> {
        assert!(
            self.indices.len() % 3 == 0,
            "Number of indices is not a multiple of 3"
        );

        self.indices
            .chunks(3)
            .map(|triplet| {
                let v0 = &self.vertices[triplet[0] as usize];
                let v1 = &self.vertices[triplet[1] as usize];
                let v2 = &self.vertices[triplet[2] as usize];

                let summed_normal = Vector3::new(
                    v0.normal[0] + v1.normal[0] + v2.normal[0],
                    v0.normal[1] + v1.normal[1] + v2.normal[1],
                    v0.normal[2] + v1.normal[2] + v2.normal[2],
                );

                Triangle {
                    vertices: [v0.position, v1.position, v2.position],
                    normal: summed_normal.normalize().into(),
                }
            })
            .collect()
    }

    pub fn import_stl<P: AsRef<OsStr>, Processor: StlProcessorTrait>(
        &mut self,
        filename: P,
        processor: &Processor,
    ) {
        let imported_triangles: Vec<Triangle> = processor
            .read_stl(filename.as_ref())
            .expect("Error processing STL file");
        self.generate_vertices_and_indices(&imported_triangles);
        self.generate_simple_vertices_and_indices(&imported_triangles);
        self.get_triangles_for_slicing();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use stl_io::Triangle;

    #[test]
    fn test_default() {
        let mesh = Mesh::default();

        assert!(mesh.vertices.is_empty(), "Default vertices should be empty");
        assert!(mesh.indices.is_empty(), "Default indices should be empty");
    }

    #[test]
    fn test_single_triangle_normal() {
        // Create a mesh with a single triangle lying on the XY-plane
        let mesh = Mesh {
            vertices: vec![
                Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    barycentric: [1.0, 0.0, 0.0],
                },
                Vertex {
                    position: [1.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    barycentric: [0.0, 1.0, 0.0],
                },
                Vertex {
                    position: [0.0, 1.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    barycentric: [0.0, 0.0, 1.0],
                },
            ],
            indices: vec![0, 1, 2],
            simple_indices: Vec::new(),
            simple_vertices: Vec::new(),
        };

        let triangles: Vec<Triangle> = mesh.into_triangle_vec();

        assert_eq!(triangles.len(), 1, "There should be exactly one triangle.");

        let expected_normal = [0.0, 0.0, 1.0];
        let calculated_normal = triangles[0].normal;

        for i in 0..3 {
            assert!(
                (calculated_normal[i] - expected_normal[i]).abs() < 1e-5,
                "Normal component {} does not match expected value.",
                i
            );
        }
    }

    #[test]
    fn test_multiple_triangles_normals() {
        // Create a mesh with two triangles forming a square on the XY-plane
        let mesh = Mesh {
            vertices: vec![
                Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    barycentric: [1.0, 0.0, 0.0],
                },
                Vertex {
                    position: [1.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    barycentric: [0.0, 1.0, 0.0],
                },
                Vertex {
                    position: [1.0, 1.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    barycentric: [0.0, 0.0, 1.0],
                },
                Vertex {
                    position: [0.0, 1.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    barycentric: [0.0, 1.0, 0.0],
                },
            ],
            indices: vec![0, 1, 2, 0, 2, 3],
            simple_indices: Vec::new(),
            simple_vertices: Vec::new(),
        };

        let triangles: Vec<Triangle> = mesh.into_triangle_vec();

        assert_eq!(triangles.len(), 2, "There should be exactly two triangles.");

        let expected_normal = [0.0, 0.0, 1.0];
        for (i, triangle) in triangles.iter().enumerate() {
            for j in 0..3 {
                assert!(
                    (triangle.normal[j] - expected_normal[j]).abs() < 1e-5,
                    "Triangle {}: Normal component {} does not match expected value.",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_degenerate_triangle_normal() {
        // Create a mesh with a degenerate triangle (all vertices have the same position)
        let mesh = Mesh {
            vertices: vec![
                Vertex {
                    position: [1.0, 1.0, 1.0],
                    normal: [1.0, 0.0, 0.0],
                    barycentric: [1.0, 0.0, 0.0],
                },
                Vertex {
                    position: [1.0, 1.0, 1.0],
                    normal: [0.0, 1.0, 0.0],
                    barycentric: [0.0, 1.0, 0.0],
                },
                Vertex {
                    position: [1.0, 1.0, 1.0],
                    normal: [0.0, 0.0, 1.0],
                    barycentric: [0.0, 0.0, 1.0],
                },
            ],
            indices: vec![0, 1, 2],
            simple_indices: Vec::new(),
            simple_vertices: Vec::new(),
        };

        let triangles: Vec<Triangle> = mesh.into_triangle_vec();

        assert_eq!(triangles.len(), 1, "There should be exactly one triangle.");

        // The summed normal is [1,1,1], normalized to [1/sqrt(3), 1/sqrt(3), 1/sqrt(3)]
        let expected_normal = [
            1.0 / (3.0_f32).sqrt(),
            1.0 / (3.0_f32).sqrt(),
            1.0 / (3.0_f32).sqrt(),
        ];
        let calculated_normal = triangles[0].normal;

        for i in 0..3 {
            assert!(
                (calculated_normal[i] - expected_normal[i]).abs() < 1e-5,
                "Normal component {} does not match expected value for degenerate triangle.",
                i
            );
        }
    }

    #[test]
    fn test_non_uniform_vertex_normals() {
        // Create a mesh with a single triangle with non-uniform vertex normals
        let mesh = Mesh {
            vertices: vec![
                Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [1.0, 0.0, 0.0],
                    barycentric: [1.0, 0.0, 0.0],
                },
                Vertex {
                    position: [1.0, 0.0, 0.0],
                    normal: [0.0, 1.0, 0.0],
                    barycentric: [0.0, 1.0, 0.0],
                },
                Vertex {
                    position: [0.0, 1.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    barycentric: [0.0, 0.0, 1.0],
                },
            ],
            indices: vec![0, 1, 2],
            simple_indices: Vec::new(),
            simple_vertices: Vec::new(),
        };

        let triangles: Vec<Triangle> = mesh.into_triangle_vec();

        assert_eq!(triangles.len(), 1, "There should be exactly one triangle.");

        // The summed normal is [1,1,1], normalized to [1/sqrt(3), 1/sqrt(3), 1/sqrt(3)]
        let expected_normal = [
            1.0 / (3.0_f32).sqrt(),
            1.0 / (3.0_f32).sqrt(),
            1.0 / (3.0_f32).sqrt(),
        ];
        let calculated_normal = triangles[0].normal;

        for i in 0..3 {
            assert!(
                (calculated_normal[i] - expected_normal[i]).abs() < 1e-5,
                "Normal component {} does not match expected value for non-uniform vertex normals.",
                i
            );
        }
    }
}
