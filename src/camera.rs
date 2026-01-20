// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use nalgebra::{Matrix4, Point3, Vector3, Vector4};

pub struct Camera {
    pub position: Point3<f32>,
    pub target: Point3<f32>,
    up: Vector3<f32>,
    yaw: f32,         // Rotation around the Y axis
    pitch: f32,       // Rotation around the X axis
    sensitivity: f32, // Mouse sensitivity
    distance: f32,    // Distance from the target for zooming
    aspect_ratio: f32,
    pub projection_matrix: Matrix4<f32>,
}

impl Camera {
    pub fn new(aspect_ratio: f32) -> Self {
        let mut camera: Camera = Self {
            position: Point3::new(0.0, 0.0, 0.0),
            target: Point3::new(0.0, 0.0, 0.0),
            up: Vector3::new(0.0, 0.0, -1.0),
            yaw: -135.0,
            pitch: -45.0,
            sensitivity: 0.1, // Adjust as needed for mouse sensitivity
            distance: 100.0,  // Initial distance from the target
            aspect_ratio,
            projection_matrix: Self::projection_matrix(aspect_ratio),
        };
        camera.update_camera_position();
        camera
    }

    pub fn set_aspect_ratio(&mut self, aspect_ratio: f32) {
        if self.aspect_ratio != aspect_ratio {
            self.aspect_ratio = aspect_ratio;
            self.projection_matrix = Self::projection_matrix(aspect_ratio);
        }
    }

    /// Returns the view matrix calculated using LookAt.
    pub fn view_matrix(&self) -> Matrix4<f32> {
        Matrix4::look_at_rh(&self.position, &self.target, &self.up)
    }

    /// Returns the projection matrix using a perspective projection.
    fn projection_matrix(aspect_ratio: f32) -> Matrix4<f32> {
        Matrix4::new_perspective(aspect_ratio, 75.0_f32.to_radians(), 0.1, 1000.0)
    }

    #[allow(dead_code)]
    pub fn view_projection_matrix(&self) -> Matrix4<f32> {
        self.view_matrix() * self.projection_matrix
    }

    #[allow(dead_code)]
    pub fn get_view_direction_vector(&self) -> Vector3<f32> {
        (self.target - self.position).normalize()
    }

    /// Processes input received from a mouse input system.
    /// Expects the offset value in both the x and y direction.
    pub fn pitch_yaw(&mut self, delta_x: f32, delta_y: f32) {
        self.yaw += delta_x * self.sensitivity;
        self.pitch += delta_y * self.sensitivity;

        self.pitch = self.pitch.clamp(-89.9, 89.9);

        // Update camera position based on new yaw and pitch
        self.update_camera_position();
    }

    // Handle pan events
    pub fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let right = self.right();

        self.target -= (right * delta_x * self.sensitivity)
            * (self.distance * (self.sensitivity * self.sensitivity));
        self.target -= (self.up() * delta_y * self.sensitivity)
            * (self.distance * (self.sensitivity * self.sensitivity));

        self.update_camera_position();
    }

    /// Updates the camera position based on yaw and pitch angles, while keeping the target fixed.
    fn update_camera_position(&mut self) {
        // Convert angles to radians
        let yaw_rad = self.yaw.to_radians();
        let pitch_rad = self.pitch.to_radians();

        // Calculate the new direction vector
        let direction = Vector3::new(
            yaw_rad.cos() * pitch_rad.cos(),
            yaw_rad.sin() * pitch_rad.cos(),
            pitch_rad.sin(),
        )
        .normalize();

        // Update position based on the direction and distance
        self.position = self.target - (direction * self.distance);
    }

    /// Zooms the camera in or out by adjusting the distance from the target.
    pub fn zoom(&mut self, delta: f32) {
        self.distance -= delta * self.sensitivity;
        self.distance = self.distance.clamp(10.0, 300.0);
        self.update_camera_position();
    }

    /// Calculates the right vector based on the current view.
    fn right(&self) -> Vector3<f32> {
        (self.target - self.position).cross(&self.up).normalize()
    }

    fn up(&self) -> Vector3<f32> {
        let forward = (self.target - self.position).normalize();
        self.right().cross(&forward).normalize() // Get the up direction relative to the camera's view
    }

    #[allow(dead_code)]
    fn screen_to_ndc(x: f32, y: f32, width: u32, height: u32) -> (f32, f32) {
        let ndc_x = (2.0 * x) / width as f32 - 1.0;
        let ndc_y = 1.0 - (2.0 * y) / height as f32; // Flip y for OpenGL
        (ndc_x, ndc_y)
    }

    #[allow(dead_code)]
    fn get_ray_from_camera(
        mouse_x: f32,
        mouse_y: f32,
        width: u32,
        height: u32,
        view_proj: &Matrix4<f32>,
    ) -> Option<(Vector3<f32>, Vector3<f32>)> {
        // Convert screen coordinates to NDC
        let (ndc_x, ndc_y) = Self::screen_to_ndc(mouse_x, mouse_y, width, height);

        // NDC coordinates at near and far planes
        let ndc_near = Vector4::new(ndc_x, ndc_y, -1.0, 1.0);
        let ndc_far = Vector4::new(ndc_x, ndc_y, 1.0, 1.0);

        // Compute inverse of view-projection matrix
        let inv_view_proj = view_proj.try_inverse()?;

        // Unproject to world space
        let world_near = inv_view_proj * ndc_near;
        let world_far = inv_view_proj * ndc_far;

        // Perform perspective divide
        let world_near = world_near.xyz() / world_near.w;
        let world_far = world_far.xyz() / world_far.w;

        // Compute ray direction
        let ray_direction = (world_far - world_near).normalize();

        Some((world_near, ray_direction))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::relative_eq;
    use nalgebra::{Matrix4, Point3, Vector3};

    const EPSILON: f32 = 1e-2;

    #[test]
    fn test_new() {
        let aspect_ratio = 16.0 / 9.0;
        let camera = Camera::new(aspect_ratio);

        // Expected position based on calculations
        let expected_position = Point3::new(50.0, 50.0, 70.71068);

        assert!(
            relative_eq!(camera.position, expected_position, epsilon = EPSILON),
            "Camera position mismatch: expected {:?}, got {:?}",
            expected_position,
            camera.position
        );

        assert_eq!(camera.target, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(camera.up, Vector3::new(0.0, 0.0, -1.0));
        assert_eq!(camera.yaw, -135.0);
        assert_eq!(camera.pitch, -45.0);
        assert_eq!(camera.sensitivity, 0.1);
        assert_eq!(camera.distance, 100.0);

        // Test projection matrix parameters
        let expected_projection =
            Matrix4::new_perspective(aspect_ratio, 75.0_f32.to_radians(), 0.1, 1000.0);
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    relative_eq!(
                        camera.projection_matrix[(i, j)],
                        expected_projection[(i, j)],
                        epsilon = EPSILON
                    ),
                    "Projection matrix mismatch at ({}, {})",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_view_matrix() {
        let camera = Camera::new(16.0 / 9.0);
        let view = camera.view_matrix();

        // Expected view matrix can be computed manually or compared with nalgebra's look_at_rh
        let expected_view = Matrix4::look_at_rh(&camera.position, &camera.target, &camera.up);

        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    relative_eq!(view[(i, j)], expected_view[(i, j)], epsilon = EPSILON),
                    "View matrix mismatch at ({}, {})",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_projection_matrix() {
        let aspect_ratio = 16.0 / 9.0;
        let projection = Camera::projection_matrix(aspect_ratio);

        let expected_projection =
            Matrix4::new_perspective(aspect_ratio, 75.0_f32.to_radians(), 0.1, 1000.0);

        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    relative_eq!(
                        projection[(i, j)],
                        expected_projection[(i, j)],
                        epsilon = EPSILON
                    ),
                    "Projection matrix mismatch at ({}, {})",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_view_projection_matrix() {
        let camera = Camera::new(16.0 / 9.0);
        let vp = camera.view_projection_matrix();

        let expected_vp = camera.view_matrix() * camera.projection_matrix;

        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    relative_eq!(vp[(i, j)], expected_vp[(i, j)], epsilon = EPSILON),
                    "ViewProjection matrix mismatch at ({}, {})",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_get_view_direction_vector() {
        let camera = Camera::new(16.0 / 9.0);
        let view_dir = camera.get_view_direction_vector();

        let expected_dir = (camera.target - camera.position).normalize();

        assert!(
            relative_eq!(view_dir, expected_dir, epsilon = EPSILON),
            "View direction vector mismatch"
        );
    }

    #[test]
    fn test_pitch_yaw() {
        let mut camera = Camera::new(16.0 / 9.0);
        let initial_yaw = camera.yaw;
        let initial_pitch = camera.pitch;

        // Apply pitch and yaw changes
        let delta_x = 10.0;
        let delta_y = -20.0;
        camera.pitch_yaw(delta_x, delta_y);

        // Expected new yaw and pitch
        let expected_yaw = initial_yaw + delta_x * camera.sensitivity;
        let expected_pitch_unclamped = initial_pitch + delta_y * camera.sensitivity;
        let expected_pitch = expected_pitch_unclamped.max(-89.9).min(89.9);

        assert!(
            relative_eq!(camera.yaw, expected_yaw, epsilon = EPSILON),
            "Yaw not updated correctly: expected {}, got {}",
            expected_yaw,
            camera.yaw
        );
        assert!(
            relative_eq!(camera.pitch, expected_pitch, epsilon = EPSILON),
            "Pitch not updated correctly: expected {}, got {}",
            expected_pitch,
            camera.pitch
        );

        // Ensure pitch upper constraint
        camera.pitch_yaw(0.0, -100000.0); // Attempt to set pitch beyond 89.9
        assert!(
            relative_eq!(camera.pitch, -89.9, epsilon = EPSILON),
            "Pitch upper constraint not enforced: expected 89.9, got {}",
            camera.pitch
        );

        // Ensure pitch lower constraint
        camera.pitch_yaw(0.0, 100000.0); // Attempt to set pitch below -89.9
        assert!(
            relative_eq!(camera.pitch, 89.9, epsilon = EPSILON),
            "Pitch lower constraint not enforced: expected -89.9, got {}",
            camera.pitch
        );
    }

    #[test]
    fn test_pan() {
        let mut camera = Camera::new(16.0 / 9.0);
        let initial_target = camera.target;

        // Define pan deltas
        let delta_x = 5.0;
        let delta_y = -3.0;

        // Apply pan
        camera.pan(delta_x, delta_y);

        // Calculate expected target movement
        let right = camera.right();
        let up = camera.up();

        let movement_scale = camera.distance * (camera.sensitivity * camera.sensitivity);
        let expected_target = initial_target
            - (right * delta_x * camera.sensitivity) * movement_scale
            - (up * delta_y * camera.sensitivity) * movement_scale;

        assert!(
            relative_eq!(camera.target, expected_target, epsilon = EPSILON),
            "Target not panned correctly: expected {:?}, got {:?}",
            expected_target,
            camera.target
        );

        // Position should be updated to maintain the distance from the new target
        let expected_position =
            expected_target - (camera.get_view_direction_vector() * camera.distance);
        assert!(
            relative_eq!(camera.position, expected_position, epsilon = EPSILON),
            "Position not updated correctly after panning: expected {:?}, got {:?}",
            expected_position,
            camera.position
        );
    }

    #[test]
    fn test_zoom() {
        let mut camera = Camera::new(16.0 / 9.0);
        let initial_distance = camera.distance;

        // Apply zoom in
        let delta_zoom_in = 50.0;
        camera.zoom(delta_zoom_in);
        let expected_distance_in = initial_distance - (delta_zoom_in * camera.sensitivity);
        assert!(
            relative_eq!(camera.distance, expected_distance_in, epsilon = EPSILON),
            "Distance not zoomed in correctly: expected {}, got {}",
            expected_distance_in,
            camera.distance
        );

        // Apply zoom out within limits
        let delta_zoom_out = -200.0;
        camera.distance = initial_distance;
        camera.zoom(delta_zoom_out);
        let expected_distance_out = initial_distance - (delta_zoom_out * camera.sensitivity);
        assert!(
            relative_eq!(camera.distance, expected_distance_out, epsilon = EPSILON),
            "Distance not zoomed out correctly: expected {}, got {}",
            expected_distance_out,
            camera.distance
        );

        // Apply zoom out beyond maximum limit
        camera.zoom(-1000000.0); // Attempt to exceed maximum
        assert!(
            relative_eq!(camera.distance, 300.0, epsilon = EPSILON),
            "Distance upper limit not enforced: expected 300.0, got {}",
            camera.distance
        );

        // Apply zoom in beyond minimum limit
        camera.zoom(1000000.0); // Attempt to go below minimum
        assert!(
            relative_eq!(camera.distance, 10.0, epsilon = EPSILON),
            "Distance lower limit not enforced: expected 10.0, got {}",
            camera.distance
        );
    }

    #[test]
    fn test_right_vector() {
        let camera = Camera::new(16.0 / 9.0);
        let right = camera.right();

        // The right vector should be perpendicular to the view direction and up vector
        let view_dir = camera.get_view_direction_vector();
        let up = camera.up();

        // Check orthogonality
        assert!(
            relative_eq!(view_dir.dot(&right), 0.0, epsilon = EPSILON),
            "Right vector not orthogonal to view direction"
        );
        assert!(
            relative_eq!(up.dot(&right), 0.0, epsilon = EPSILON),
            "Right vector not orthogonal to up vector"
        );

        // Check normalization
        assert!(
            relative_eq!(right.norm(), 1.0, epsilon = EPSILON),
            "Right vector is not normalized"
        );
    }

    #[test]
    fn test_up_vector() {
        let camera = Camera::new(16.0 / 9.0);
        let up = camera.up();

        // The up vector should be perpendicular to the view direction and right vector
        let view_dir = camera.get_view_direction_vector();
        let right = camera.right();

        // Check orthogonality
        assert!(
            relative_eq!(view_dir.dot(&up), 0.0, epsilon = EPSILON),
            "Up vector not orthogonal to view direction"
        );
        assert!(
            relative_eq!(right.dot(&up), 0.0, epsilon = EPSILON),
            "Up vector not orthogonal to right vector"
        );

        // Check normalization
        assert!(
            relative_eq!(up.norm(), 1.0, epsilon = EPSILON),
            "Up vector is not normalized"
        );
    }

    #[test]
    fn test_screen_to_ndc() {
        // Case 1: Center of the screen
        let (ndc_x, ndc_y) = Camera::screen_to_ndc(960.0, 540.0, 1920, 1080);
        assert!((ndc_x - 0.0).abs() < 1e-6);
        assert!((ndc_y - 0.0).abs() < 1e-6);

        // Case 2: Top-left corner
        let (ndc_x, ndc_y) = Camera::screen_to_ndc(0.0, 0.0, 1920, 1080);
        assert!((ndc_x + 1.0).abs() < 1e-6);
        assert!((ndc_y - 1.0).abs() < 1e-6);

        // Case 3: Bottom-right corner
        let (ndc_x, ndc_y) = Camera::screen_to_ndc(1920.0, 1080.0, 1920, 1080);
        assert!((ndc_x - 1.0).abs() < 1e-6);
        assert!((ndc_y + 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_get_ray_from_camera() {
        // Case 1: Basic setup with an identity matrix
        let view_proj = Matrix4::identity();

        // Mouse at center of screen
        let result = Camera::get_ray_from_camera(960.0, 540.0, 1920, 1080, &view_proj);
        assert!(result.is_some());

        if let Some((ray_origin, ray_direction)) = result {
            // The ray origin for an identity view_proj should be at the near plane (0,0,-1)
            assert!((ray_origin - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-6);
            // The direction should be pointing straight down the Z-axis in NDC space
            assert!((ray_direction - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-6);
        }

        // Case 2: With a custom view projection matrix
        let view_proj = Matrix4::new_orthographic(-1.0, 1.0, -1.0, 1.0, 0.1, 100.0);

        // Mouse at the top-left corner
        let result = Camera::get_ray_from_camera(0.0, 0.0, 1920, 1080, &view_proj);
        assert!(result.is_some());

        if let Some((ray_origin, ray_direction)) = result {
            // Check if ray origin and direction are reasonable
            assert!(ray_origin.norm() > 0.0);
            assert!(ray_direction.norm() > 0.0);
        }
    }
}
