use glam::{Mat4, Vec3};

#[derive(Debug, Clone, Copy)]
pub struct FrustumPlane {
    pub normal: Vec3,
    pub distance: f32,
}

impl FrustumPlane {
    fn from_coeffs(a: f32, b: f32, c: f32, d: f32) -> Self {
        let normal = Vec3::new(a, b, c);
        let length = normal.length();
        FrustumPlane {
            normal: normal / length,
            distance: d / length,
        }
    }
}
pub struct Frustum {
    pub planes: [FrustumPlane; 6],
}

impl Frustum {
    pub fn from_view_proj(m: &Mat4) -> Self {
        let m = m.to_cols_array(); // column-major
        let planes = [
            // Left
            FrustumPlane::from_coeffs(m[3] + m[0], m[7] + m[4], m[11] + m[8], m[15] + m[12]),
            // Right
            FrustumPlane::from_coeffs(m[3] - m[0], m[7] - m[4], m[11] - m[8], m[15] - m[12]),
            // Bottom
            FrustumPlane::from_coeffs(m[3] + m[1], m[7] + m[5], m[11] + m[9], m[15] + m[13]),
            // Top
            FrustumPlane::from_coeffs(m[3] - m[1], m[7] - m[5], m[11] - m[9], m[15] - m[13]),
            // Near
            FrustumPlane::from_coeffs(m[3] + m[2], m[7] + m[6], m[11] + m[10], m[15] + m[14]),
            // Far
            FrustumPlane::from_coeffs(m[3] - m[2], m[7] - m[6], m[11] - m[10], m[15] - m[14]),
        ];
        Self { planes }
    }

    pub fn intersects_sphere(&self, center: Vec3, radius: f32) -> bool {
        for plane in &self.planes {
            if plane.normal.dot(center) + plane.distance < -radius {
                return false; // completely outside
            }
        }
        true
    }
}
