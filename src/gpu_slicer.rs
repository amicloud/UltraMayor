// gpu_slicer.rs
// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use image::{ImageBuffer, Luma};
use std::cell::RefCell;
use std::fs;
use std::rc::Rc;
use stl_io::Triangle;

use glow::Context as GlowContext;
use std::error::Error;

use crate::body::Body;
pub struct GPUSlicer {
    gl: Rc<GlowContext>,
}

impl GPUSlicer {
    pub fn new(gl: Rc<GlowContext>) -> Self {
        Self { gl }
    }

    pub fn slice_bodies(
        &self,
        _bodies: Vec<Rc<RefCell<Body>>>,
    ) -> Result<Vec<ImageBuffer<Luma<u8>, Vec<u8>>>, Box<dyn std::error::Error>> {
        let triangles: Vec<Triangle> = Vec::new();
        self.generate_slice_images(&triangles)
    }
    // Function to generate slice images
    fn generate_slice_images(
        &self,
        triangles: &[Triangle],
    ) -> Result<Vec<ImageBuffer<Luma<u8>, Vec<u8>>>, Box<dyn Error>> {
        let gl = &self.gl;

        // Read and compile the compute shader
        let shader_source = fs::read_to_string("shaders/slicer_shader.glsl")?;
        Ok(Vec::new())
    }
}
