// src/body.rs
// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use std::ffi::OsStr;
use std::path::Path;

use crate::stl_processor::StlProcessorTrait;
use crate::{material::Material, mesh::Mesh};
use nalgebra::{Matrix4, Quaternion, UnitQuaternion, Vector3};
use slint::SharedString;
use uuid::Uuid;
#[allow(dead_code)]
#[derive(Default, Clone)]
pub struct AABB {
    min: Vector3<f32>,
    max: Vector3<f32>,
}

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

#[derive(Clone)]
pub struct Body {
    pub position: Vector3<f32>,
    pub rotation: Quaternion<f32>,
    pub scale: Vector3<f32>,
    pub mesh: Mesh,
    pub enabled: bool,
    pub selected: bool,
    pub name: String,
    pub visible: bool,
    pub uuid: Uuid,
    pub aabb: AABB,
    pub material: Material,
    pub display_in_ui_list: bool,
    pub selectable: bool,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            position: Vector3::zeros(),
            rotation: Quaternion::identity(),
            scale: Vector3::new(1.0, 1.0, 1.0),
            mesh: Mesh::default(),
            enabled: true,
            selected: true,
            name: "".to_string(),
            visible: true,
            uuid: Uuid::new_v4(),
            aabb: AABB::default(),
            material: Material::default_resin(),
            display_in_ui_list: true,
            selectable: true,
        }
    }
}

impl PartialEq for Body {
    fn eq(&self, other: &Body) -> bool {
        self.uuid == other.uuid
    }
}

impl Body {
    pub fn new(mesh: Mesh) -> Self {
        let mut b = Body::default();
        b.mesh = mesh;
        b
    }

    #[allow(dead_code)]
    pub fn eq_uuid(&self, other: &Uuid) -> bool {
        self.uuid == *other
    }

    pub fn eq_uuid_ss(&self, other: &SharedString) -> bool {
        self.uuid.to_string() == *other.to_string()
    }

    pub fn new_from_stl<P: AsRef<OsStr>, Processor: StlProcessorTrait>(
        filename: P,
        processor: &Processor,
    ) -> Self {
        let mut body = Body::default();
        let path = Path::new(filename.as_ref());
        body.name = path
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned();
        body.mesh.import_stl(filename, processor);
        body.aabb = AABB::from_vertices(&body.mesh.vertices);
        body.translate(Vector3::new(0.0, 0.0, body.aabb.min.z * -1.0));
        body
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
    #[allow(dead_code)]
    pub fn translate(&mut self, val: Vector3<f32>) {
        self.position += val;
    }
    #[allow(dead_code)]
    pub fn rotate(&mut self, _val: Vector3<f32>) {
        todo!("Implement");
    }
    #[allow(dead_code)]
    pub fn scale(&mut self, _val: Vector3<f32>) {
        todo!("Implement");
    }

    pub fn set_position(&mut self, position: Vector3<f32>) {
        self.position = position;
    }

    pub fn set_rotation(&mut self, rotation: Vector3<f32>) {
        self.rotation = Self::euler_to_quaternion(rotation);
    }

    pub fn set_rotation_quat(&mut self, rotation: Quaternion<f32>) {
        self.rotation = rotation;
    }

    pub fn set_scale(&mut self, scale: Vector3<f32>) {
        self.scale = scale;
    }
    pub fn euler_to_quaternion(euler: Vector3<f32>) -> Quaternion<f32> {
        // Convert Euler angles (in degrees) to radians
        // convert to f64 for more accuracy during calculations, hopefully
        let roll = (euler.x as f64).to_radians();
        let pitch = (euler.y as f64).to_radians();
        let yaw = (euler.z as f64).to_radians();

        // Compute half angles
        let (sin_roll, cos_roll) = (roll / 2.0).sin_cos();
        let (sin_pitch, cos_pitch) = (pitch / 2.0).sin_cos();
        let (sin_yaw, cos_yaw) = (yaw / 2.0).sin_cos();

        // Compute the quaternion from the Euler angles
        let w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
        let x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
        let y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
        let z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
        Quaternion::new(
            w as f32, // w component
            x as f32, // x component
            y as f32, // y component
            z as f32, // z component
        )
    }

    pub fn quaternion_to_euler(quat: &Quaternion<f32>) -> Vector3<f32> {
        let w = quat.w;
        let x = quat.i;
        let y = quat.j;
        let z = quat.k;

        // Calculate the roll (x-axis rotation)
        let sinr_cosp = 2.0 * (w * x + y * z);
        let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Calculate the pitch (y-axis rotation)
        let sinp = 2.0 * (w * y - z * x);
        let pitch = if sinp.abs() >= 1.0 {
            // Use 90 degrees if out of range
            sinp.signum() * std::f32::consts::FRAC_PI_2
        } else {
            sinp.asin()
        };

        // Calculate the yaw (z-axis rotation)
        let siny_cosp = 2.0 * (w * z + x * y);
        let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        // Convert from radians to degrees
        Vector3::new(roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::Vertex;
    use crate::stl_processor::StlProcessorTrait;
    use approx::relative_eq;
    use nalgebra::{Matrix4, UnitQuaternion, Vector3};
    use stl_io::Triangle;

    const EPSILON: f32 = 1e-4;

    // Mock implementation of StlProcessorTrait without cloning
    struct MockStlProcessor;

    impl StlProcessorTrait for MockStlProcessor {
        fn read_stl(&self, _filename: &OsStr) -> Result<Vec<Triangle>, std::io::Error> {
            Ok(vec![
                create_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
                create_triangle([1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]),
            ])
        }
    }

    // Helper function to create a triangle
    fn create_triangle(v0: [f32; 3], v1: [f32; 3], v2: [f32; 3]) -> Triangle {
        Triangle {
            normal: [0.0, 0.0, 1.0],
            vertices: [v0, v1, v2],
        }
    }

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
    fn test_new_from_stl() {
        // Arrange: Create a mock processor
        let mock_processor = MockStlProcessor;

        // Act: Create Body from STL using mock processor
        let body = Body::new_from_stl("dummy_filename.stl", &mock_processor);

        // Additionally, check that vertices and indices are generated correctly
        let expected_vertices = vec![
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
            Vertex {
                position: [1.0, 0.0, 0.0],
                normal: [0.0, 0.0, 1.0],
                barycentric: [1.0, 0.0, 0.0],
            },
            Vertex {
                position: [1.0, 1.0, 0.0],
                normal: [0.0, 0.0, 1.0],
                barycentric: [0.0, 1.0, 0.0],
            },
        ];

        let expected_indices = vec![0, 1, 2, 3, 4, 2];
        println!("{:?}", &body.mesh.vertices);
        println!();
        println!("{:?}", &expected_vertices);
        assert_eq!(
            body.mesh.vertices.len(),
            expected_vertices.len(),
            "Mesh should have the 5 after import due to barycentric, instead of 4 without the barycentric"
        );

        for (imported, expected) in body.mesh.vertices.iter().zip(expected_vertices.iter()) {
            let imported_pos = Vector3::from(imported.position);
            let expected_pos = Vector3::from(expected.position);
            assert!(
                relative_eq!(imported_pos, expected_pos, epsilon = EPSILON),
                "Vertex position mismatch. Expected {:?}, got {:?}",
                expected.position,
                imported.position
            );

            let imported_norm = Vector3::from(imported.normal);
            let expected_norm = Vector3::from(expected.normal);
            assert!(
                relative_eq!(imported_norm, expected_norm, epsilon = EPSILON),
                "Vertex normal mismatch. Expected {:?}, got {:?}",
                expected.normal,
                imported.normal
            );
        }

        assert_eq!(
            body.mesh.indices, expected_indices,
            "Mesh indices do not match expected after import"
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
            selected: true,
            enabled: true,
            name: "".to_string(),
            visible: true,
            uuid: Uuid::new_v4(),
            aabb: AABB::default(),
            material: Material::default_resin(),
            display_in_ui_list: true,
            selectable: true,
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

    #[test]
    fn test_euler_to_quaternion_basic() {
        // Test case: Euler angles (45°, 30°, 60°)
        let euler_angles = Vector3::new(45.0, 30.0, 60.0);
        let quat = Body::euler_to_quaternion(euler_angles);

        // Expected quaternion from these angles can be computed manually or using a reference system
        let expected_quat = Quaternion::new(0.82236, 0.20056, 0.39190, 0.36042);

        // Check that the quaternion is close to the expected result
        assert!((quat.w - expected_quat.w).abs() < 1e-5);
        assert!((quat.i - expected_quat.i).abs() < 1e-5);
        assert!((quat.j - expected_quat.j).abs() < 1e-5);
        assert!((quat.k - expected_quat.k).abs() < 1e-5);
    }

    #[test]
    fn test_euler_to_quaternion_identity() {
        // Test case: Euler angles (0°, 0°, 0°) should result in identity quaternion
        let euler_angles = Vector3::new(0.0, 0.0, 0.0);
        let quat = Body::euler_to_quaternion(euler_angles);

        // Identity quaternion (no rotation)
        let expected_quat: Quaternion<f32> = Quaternion::identity();

        assert!((quat.w - expected_quat.w).abs() < 1e-5);
        assert!((quat.i - expected_quat.i).abs() < 1e-5);
        assert!((quat.j - expected_quat.j).abs() < 1e-5);
        assert!((quat.k - expected_quat.k).abs() < 1e-5);
    }

    #[test]
    fn test_euler_to_quaternion_90_deg() {
        // Test case: Euler angles (90°, 0°, 0°)
        let euler_angles = Vector3::new(90.0, 0.0, 0.0);
        let quat = Body::euler_to_quaternion(euler_angles);

        // Expected quaternion for 90-degree X-axis rotation
        let expected_quat = Quaternion::new(0.7071, 0.7071, 0.0, 0.0);

        assert!((quat.w - expected_quat.w).abs() < 1e-5);
        assert!((quat.i - expected_quat.i).abs() < 1e-5);
        assert!((quat.j - expected_quat.j).abs() < 1e-5);
        assert!((quat.k - expected_quat.k).abs() < 1e-5);
    }

    #[test]
    fn test_quaternion_to_euler_basic() {
        // Test case: Quaternion from Euler angles (45°, 30°, 60°)
        let quat = Quaternion::new(0.82236, 0.20056, 0.39190, 0.36042);
        let euler = Body::quaternion_to_euler(&quat);

        // Expected Euler angles (in degrees)
        let expected_euler = Vector3::new(45.0, 30.0, 60.0);

        // Check that the Euler angles are close to the expected result
        assert!((euler.x - expected_euler.x).abs() < 1e-1); // Tolerance due to floating-point precision
        assert!((euler.y - expected_euler.y).abs() < 1e-1);
        assert!((euler.z - expected_euler.z).abs() < 1e-1);
    }

    #[test]
    fn test_quaternion_to_euler_identity() {
        // Test case: Identity quaternion should result in (0°, 0°, 0°) Euler angles
        let quat = Quaternion::identity();
        let euler = Body::quaternion_to_euler(&quat);

        // Expected Euler angles (in degrees)
        let expected_euler = Vector3::new(0.0, 0.0, 0.0);

        assert!((euler.x - expected_euler.x).abs() < 1e-5);
        assert!((euler.y - expected_euler.y).abs() < 1e-5);
        assert!((euler.z - expected_euler.z).abs() < 1e-5);
    }

    #[test]
    fn test_quaternion_to_euler_90_deg() {
        // Test case: Quaternion for a 90° rotation around the X-axis
        let quat = Quaternion::new(0.7071, 0.7071, 0.0, 0.0);
        let euler = Body::quaternion_to_euler(&quat);

        // Expected Euler angles (in degrees)
        let expected_euler = Vector3::new(90.0, 0.0, 0.0);

        assert!((euler.x - expected_euler.x).abs() < 1e-1); // Tolerance due to precision
        assert!((euler.y - expected_euler.y).abs() < 1e-1);
        assert!((euler.z - expected_euler.z).abs() < 1e-1);
    }
}
