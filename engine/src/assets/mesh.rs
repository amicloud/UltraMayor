// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.
use approx::relative_eq;
use bytemuck::{Pod, Zeroable};
use glam::Vec3;
use std::hash::{Hash, Hasher};

use crate::components::collider_component::{BVHNode, Triangle};

#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
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

unsafe impl Pod for Vertex {}
impl Eq for Vertex {}

impl Hash for Vertex {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.position_bits().hash(state);
        self.normal_bits().hash(state);
        self.barycentric_bits().hash(state);
    }
}

impl Vertex {
    pub(crate) fn stride() -> i32 {
        std::mem::size_of::<Vertex>() as i32
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
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    #[allow(dead_code)]
    fn intersect_ray(&self, ray_origin: Vec3, ray_dir: Vec3) -> bool {
        let inv_dir = Vec3::new(1.0 / ray_dir.x, 1.0 / ray_dir.y, 1.0 / ray_dir.z);

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

    pub(crate) fn from_vertices(vertices: &[Vertex]) -> Self {
        // Initialize min and max with the first vertex
        let mut min = vertices[0];
        let mut max = vertices[0];

        // Iterate over all vertices to find min and max values
        vertices.iter().for_each(|vertex| {
            min.position[0] = min.position[0].min(vertex.position[0]);
            min.position[1] = min.position[1].min(vertex.position[1]);
            min.position[2] = min.position[2].min(vertex.position[2]);

            max.position[0] = max.position[0].max(vertex.position[0]);
            max.position[1] = max.position[1].max(vertex.position[1]);
            max.position[2] = max.position[2].max(vertex.position[2]);
        });

        Aabb {
            min: min.position.into(),
            max: max.position.into(),
        }
    }

    pub fn union(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    pub fn area(&self) -> f32 {
        let d = self.max - self.min;
        2.0 * (d.x * d.y + d.x * d.z + d.y * d.z)
    }

    pub fn intersects(&self, other: &Aabb) -> bool {
        !(self.max.x < other.min.x
            || self.min.x > other.max.x
            || self.max.y < other.min.y
            || self.min.y > other.max.y
            || self.max.z < other.min.z
            || self.min.z > other.max.z)
    }

    pub fn contains(&self, other: &Aabb) -> bool {
        self.min.x <= other.min.x
            && self.max.x >= other.max.x
            && self.min.y <= other.min.y
            && self.max.y >= other.max.y
            && self.min.z <= other.min.z
            && self.max.z >= other.max.z
    }
}

#[derive(Default, Clone)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,

    // Bounds
    pub aabb: Aabb,
    pub sphere_center: Vec3,
    pub sphere_radius: f32,

    // Collision
    pub bvh: Option<BVHNode>,
}

#[derive(Clone)]
pub struct GltfPrimitiveMesh {
    pub mesh: Mesh,
    pub material_index: Option<usize>,
}

impl Mesh {
    pub fn build_bvh(&mut self, max_leaf_size: usize) {
        if self.indices.len() < 3 || self.vertices.is_empty() {
            self.bvh = None;
            return;
        }

        let triangles = self.triangles_from_indices();
        if triangles.is_empty() {
            self.bvh = None;
            return;
        }

        self.bvh = Some(BVHNode::build(triangles, max_leaf_size));
    }

    fn triangles_from_indices(&self) -> Vec<Triangle> {
        let mut tris = Vec::with_capacity(self.indices.len() / 3);
        for indices in self.indices.chunks(3) {
            if indices.len() < 3 {
                continue;
            }
            let i0 = indices[0] as usize;
            let i1 = indices[1] as usize;
            let i2 = indices[2] as usize;
            if i0 >= self.vertices.len() || i1 >= self.vertices.len() || i2 >= self.vertices.len() {
                continue;
            }
            let v0 = Vec3::from(self.vertices[i0].position);
            let v1 = Vec3::from(self.vertices[i1].position);
            let v2 = Vec3::from(self.vertices[i2].position);
            tris.push(Triangle { v0, v1, v2 });
        }
        tris
    }

    pub fn compute_bounding_sphere(&mut self) {
        // Center = AABB center
        self.sphere_center = (self.aabb.min + self.aabb.max) * 0.5;
        // Radius = distance from center to farthest corner
        self.sphere_radius = (self.aabb.max - self.sphere_center).length();
    }

    pub(crate) fn compute_tangents(
        positions: &[[f32; 3]],
        normals: &[[f32; 3]],
        uvs: &[[f32; 2]],
        indices: &[u32],
    ) -> Vec<[f32; 4]> {
        let mut tan1 = vec![Vec3::new(0.0, 0.0, 0.0); positions.len()];
        let mut tan2 = vec![Vec3::new(0.0, 0.0, 0.0); positions.len()];

        let tri_count = indices.len() / 3;
        for t in 0..tri_count {
            let i0 = indices[t * 3] as usize;
            let i1 = indices[t * 3 + 1] as usize;
            let i2 = indices[t * 3 + 2] as usize;

            let p0 = Vec3::from(positions[i0]);
            let p1 = Vec3::from(positions[i1]);
            let p2 = Vec3::from(positions[i2]);

            let w0 = uvs[i0];
            let w1 = uvs[i1];
            let w2 = uvs[i2];

            let x1 = p1.x - p0.x;
            let x2 = p2.x - p0.x;
            let y1 = p1.y - p0.y;
            let y2 = p2.y - p0.y;
            let z1 = p1.z - p0.z;
            let z2 = p2.z - p0.z;

            let s1 = w1[0] - w0[0];
            let s2 = w2[0] - w0[0];
            let t1 = w1[1] - w0[1];
            let t2 = w2[1] - w0[1];

            let denom = s1 * t2 - s2 * t1;
            if denom.abs() < f32::EPSILON {
                continue;
            }
            let r = 1.0 / denom;
            let sdir = Vec3::new(
                (t2 * x1 - t1 * x2) * r,
                (t2 * y1 - t1 * y2) * r,
                (t2 * z1 - t1 * z2) * r,
            );
            let tdir = Vec3::new(
                (s1 * x2 - s2 * x1) * r,
                (s1 * y2 - s2 * y1) * r,
                (s1 * z2 - s2 * z1) * r,
            );

            tan1[i0] += sdir;
            tan1[i1] += sdir;
            tan1[i2] += sdir;

            tan2[i0] += tdir;
            tan2[i1] += tdir;
            tan2[i2] += tdir;
        }

        let mut tangents = Vec::with_capacity(positions.len());
        for i in 0..positions.len() {
            let n = Vec3::from(normals[i]);
            let t = tan1[i];

            let tangent = t - n * n.dot(t);

            let handedness = if n.cross(tangent).dot(tan2[i]) < 0.0 {
                -1.0
            } else {
                1.0
            };

            tangents.push([tangent.x, tangent.y, tangent.z, handedness]);
        }

        tangents
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default() {
        let mesh = Mesh::default();

        assert!(mesh.vertices.is_empty(), "Default vertices should be empty");
        assert!(mesh.indices.is_empty(), "Default indices should be empty");
    }

    #[test]
    fn aabb_from_vertices() {
        let vertices = vec![
            Vertex {
                position: [1.0, -2.0, 3.0],
                ..Vertex::zeroed()
            },
            Vertex {
                position: [-4.0, 5.0, -6.0],
                ..Vertex::zeroed()
            },
            Vertex {
                position: [7.0, 8.0, 9.0],
                ..Vertex::zeroed()
            },
        ];

        let aabb = Aabb::from_vertices(&vertices);
        assert_eq!(aabb.min, Vec3::new(-4.0, -2.0, -6.0));
        assert_eq!(aabb.max, Vec3::new(7.0, 8.0, 9.0));
    }

    #[test]
    fn compute_bounding_sphere() {
        let mut mesh = Mesh::default();
        mesh.vertices = vec![
            Vertex {
                position: [0.0, 0.0, 0.0],
                ..Vertex::zeroed()
            },
            Vertex {
                position: [2.0, 0.0, 0.0],
                ..Vertex::zeroed()
            },
        ];

        mesh.aabb = Aabb::from_vertices(&mesh.vertices);
        mesh.compute_bounding_sphere();

        assert_eq!(mesh.sphere_center, Vec3::new(1.0, 0.0, 0.0));
        assert!((mesh.sphere_radius - 1.0).abs() < 1e-6);
    }
}
