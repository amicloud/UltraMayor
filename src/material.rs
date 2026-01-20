use nalgebra::Vector3;
#[derive(Clone)]
pub struct Material {
    pub roughness: f32,
    pub albedo: Vector3<f32>,
    pub base_reflectance: Vector3<f32>,
    pub metallicity: f32,
    pub visualize_normals: bool,
    pub can_visualize_edges: bool,
}

impl Material {
    pub fn default_resin() -> Material {
        let reflectance_b = 0.05;
        Self {
            roughness: 0.35,
            albedo: Vector3::new(0.75, 0.75, 0.75),
            base_reflectance: Vector3::new(reflectance_b, reflectance_b, reflectance_b),
            metallicity: 0.01,
            visualize_normals: true,
            can_visualize_edges: true,
        }
    }

    pub fn build_plate() -> Material {
        let reflectance_b = 0.05;
        Self {
            roughness: 0.69,
            albedo: Vector3::new(0.1, 0.1, 0.1),
            base_reflectance: Vector3::new(reflectance_b, reflectance_b, reflectance_b),
            metallicity: 0.25,
            visualize_normals: false,
            can_visualize_edges: false,
        }
    }
}
