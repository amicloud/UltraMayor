// src/body.rs
// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use crate::{material::Material, mesh::Mesh};
use nalgebra::{Matrix4, Quaternion, UnitQuaternion, Vector3};
use slint::SharedString;
use std::ffi::OsStr;
use uuid::Uuid;
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

#[derive(Clone)]
pub struct Body {
    pub position: Vector3<f32>,
    pub rotation: Quaternion<f32>,
    pub scale: Vector3<f32>,
    pub mesh: Mesh,
    pub uuid: Uuid,
    pub material: Material,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            position: Vector3::zeros(),
            rotation: Quaternion::identity(),
            scale: Vector3::new(1.0, 1.0, 1.0),
            mesh: Mesh::default(),
            uuid: Uuid::new_v4(),
            material: Material::default_resin(),
        }
    }
}

impl PartialEq for Body {
    fn eq(&self, other: &Body) -> bool {
        self.uuid == other.uuid
    }
}

impl Body {
    #[allow(dead_code)]
    pub fn new_from_obj(path: &OsStr, position: Vector3<f32>) -> Self {
        let mut b = Body::default();
        b.mesh = Mesh::from_obj(path).unwrap();
        b.position = position;
        b
    }

    #[allow(dead_code)]
    pub fn eq_uuid(&self, other: &Uuid) -> bool {
        self.uuid == *other
    }

    #[allow(dead_code)]
    pub fn eq_uuid_ss(&self, other: &SharedString) -> bool {
        self.uuid.to_string() == *other.to_string()
    }

    pub fn get_model_matrix(&self) -> Matrix4<f32> {
        let mut model = Matrix4::identity();
        model *= Matrix4::new_translation(&self.position);
        let rotation_quat = UnitQuaternion::from_quaternion(Quaternion::new(
            self.rotation.w,
            self.rotation.i,
            self.rotation.j,
            self.rotation.k,
        ));
        model *= rotation_quat.to_homogeneous();
        model *= Matrix4::new_nonuniform_scaling(&self.scale);
        model
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::Vertex;
    use approx::relative_eq;
    use nalgebra::{Matrix4, UnitQuaternion, Vector3};

    const EPSILON: f32 = 1e-4;

    #[test]
    fn test_default() {
        let body = Body::default();

        assert_eq!(
            body.position,
            Vector3::zeros(),
            "Default position should be zero"
        );
        assert_eq!(
            body.rotation,
            Quaternion::identity(),
            "Default rotation should be identity"
        );
        assert_eq!(
            body.scale,
            Vector3::new(1.0, 1.0, 1.0),
            "Default scale should be (1.0, 1.0, 1.0)"
        );
        assert!(
            body.mesh.vertices.is_empty(),
            "Default Mesh should have no vertices"
        );
        assert!(
            body.mesh.indices.is_empty(),
            "Default Mesh should have no indices"
        );
    }

    #[test]
    fn test_get_model_matrix() {
        // Arrange: Create a Body with known position, rotation, and scale
        let position = Vector3::new(10.0, -5.0, 3.0);
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f32::consts::FRAC_PI_2); // 90 degrees around Z-axis
        let rotation_quat = rotation.quaternion();
        let scale = Vector3::new(2.0, 3.0, 4.0);

        let body = Body {
            position,
            rotation: Quaternion::new(
                rotation_quat.w,
                rotation_quat.i,
                rotation_quat.j,
                rotation_quat.k,
            ),
            scale,
            mesh: Mesh::default(),
            uuid: Uuid::new_v4(),
            material: Material::default_resin(),
        };

        // Act: Compute the model matrix
        let model_matrix = body.get_model_matrix();

        // Compute expected model matrix manually
        let translation_matrix = Matrix4::new_translation(&position);
        let rotation_matrix = rotation.to_homogeneous();
        let scaling_matrix = Matrix4::new_nonuniform_scaling(&scale);

        let expected_model = translation_matrix * rotation_matrix * scaling_matrix;

        // Assert: The computed model matrix matches the expected matrix
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    relative_eq!(
                        model_matrix[(i, j)],
                        expected_model[(i, j)],
                        epsilon = EPSILON
                    ),
                    "Model matrix element ({}, {}) mismatch. Expected {}, got {}",
                    i,
                    j,
                    expected_model[(i, j)],
                    model_matrix[(i, j)]
                );
            }
        }
    }

    #[test]
    fn test_ray_intersects_aabb() {
        let aabb = AABB {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(1.0, 1.0, 1.0),
        };

        // Case 1: Ray intersects AABB
        let ray_origin = Vector3::new(-1.0, 0.5, 0.5);
        let ray_dir = Vector3::new(1.0, 0.0, 0.0);
        assert!(aabb.intersect_ray(ray_origin, ray_dir));

        // Case 2: Ray misses AABB
        let ray_origin = Vector3::new(-1.0, 2.0, 2.0);
        let ray_dir = Vector3::new(1.0, 0.0, 0.0);
        assert!(!aabb.intersect_ray(ray_origin, ray_dir));

        // Case 3: Ray originates inside the AABB
        let ray_origin = Vector3::new(0.5, 0.5, 0.5);
        let ray_dir = Vector3::new(1.0, 0.0, 0.0);
        assert!(aabb.intersect_ray(ray_origin, ray_dir));

        // Case 4: Ray parallel to AABB and outside
        let ray_origin = Vector3::new(2.0, 0.5, 0.5);
        let ray_dir = Vector3::new(1.0, 0.0, 0.0);
        assert!(!aabb.intersect_ray(ray_origin, ray_dir));
    }

    #[test]
    fn test_aabb_from_vertices_basic() {
        // Basic case with a few vertices
        let vertices = vec![
            Vertex::new([1.0, 2.0, 3.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
            Vertex::new([4.0, 5.0, 6.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
            Vertex::new([-1.0, 0.0, 2.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
        ];

        let aabb = AABB::from_vertices(&vertices);

        // Expected values for the AABB
        assert!((aabb.min - Vector3::new(-1.0, 0.0, 2.0)).norm() < 1e-6);
        assert!((aabb.max - Vector3::new(4.0, 5.0, 6.0)).norm() < 1e-6);
    }

    #[test]
    fn test_aabb_from_vertices_single_point() {
        // Case with a single vertex
        let vertices = vec![Vertex::new(
            [1.0, 2.0, 3.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
        )];

        let aabb = AABB::from_vertices(&vertices);

        // Since there is only one vertex, min and max should both be this point
        assert!((aabb.min - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-6);
        assert!((aabb.max - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-6);
    }

    #[test]
    fn test_aabb_from_vertices_negative_coordinates() {
        // Case with vertices having negative coordinates
        let vertices = vec![
            Vertex::new([-3.0, -5.0, -2.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
            Vertex::new([1.0, 2.0, 3.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
            Vertex::new([0.0, -1.0, 2.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
        ];

        let aabb = AABB::from_vertices(&vertices);

        // Expected values for the AABB
        assert!((aabb.min - Vector3::new(-3.0, -5.0, -2.0)).norm() < 1e-6);
        assert!((aabb.max - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-6);
    }

    #[test]
    fn test_aabb_from_vertices_all_same_point() {
        // Case where all vertices are the same point
        let vertices = vec![
            Vertex::new([2.0, 2.0, 2.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
            Vertex::new([2.0, 2.0, 2.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
            Vertex::new([2.0, 2.0, 2.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
        ];

        let aabb = AABB::from_vertices(&vertices);

        // Since all points are the same, min and max should also be this point
        assert!((aabb.min - Vector3::new(2.0, 2.0, 2.0)).norm() < 1e-6);
        assert!((aabb.max - Vector3::new(2.0, 2.0, 2.0)).norm() < 1e-6);
    }

    #[test]
    fn test_aabb_from_vertices_mixed_large_range() {
        // Case with a large range of coordinates
        let vertices = vec![
            Vertex::new([100.0, 200.0, -300.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
            Vertex::new([-1000.0, -500.0, 400.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
            Vertex::new([50.0, 60.0, 70.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
        ];

        let aabb = AABB::from_vertices(&vertices);

        // Expected min and max values for the AABB
        assert!((aabb.min - Vector3::new(-1000.0, -500.0, -300.0)).norm() < 1e-6);
        assert!((aabb.max - Vector3::new(100.0, 200.0, 400.0)).norm() < 1e-6);
    }
}
