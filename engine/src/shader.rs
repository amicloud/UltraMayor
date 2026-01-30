use std::{ffi::OsStr, fs};

use glow::HasContext;

#[derive(Hash)]
pub struct Shader {
    pub program: glow::Program,
    // cached uniform locations
    pub u_view_proj_location: glow::UniformLocation,
    pub u_camera_position_location: glow::UniformLocation,
    pub u_light_direction_location: glow::UniformLocation,
    pub u_light_color_location: glow::UniformLocation,
    pub u_albedo_location: glow::UniformLocation,
    pub u_normal_location: glow::UniformLocation,
    pub u_roughness_location: glow::UniformLocation,
    pub u_base_reflectance_location: glow::UniformLocation,
    pub u_visualize_normals_location: glow::UniformLocation,
    pub u_visualize_edges_location: glow::UniformLocation,
    pub u_edge_thickness_location: glow::UniformLocation,
}

impl Shader {
    pub fn new(gl: &glow::Context, vertex_src: &OsStr, fragment_src: &OsStr) -> Self {
        unsafe {
            let vertex_shader = gl.create_shader(glow::VERTEX_SHADER).unwrap();
            let vertex_shader_source = fs::read_to_string(&vertex_src.to_str().unwrap())
                .expect("Failed to read vertex shader file");
            gl.shader_source(vertex_shader, &vertex_shader_source);
            gl.compile_shader(vertex_shader);
            if !gl.get_shader_compile_status(vertex_shader) {
                panic!(
                    "Vertex shader compilation failed: {}",
                    gl.get_shader_info_log(vertex_shader)
                );
            } else {
                println!("Vertex shader compiled successfully.");
            }

            let fragment_shader = gl.create_shader(glow::FRAGMENT_SHADER).unwrap();
            let fragment_shader_source = fs::read_to_string(&fragment_src.to_str().unwrap())
                .expect("Failed to read fragment shader file");
            gl.shader_source(fragment_shader, &fragment_shader_source);
            gl.compile_shader(fragment_shader);
            if !gl.get_shader_compile_status(fragment_shader) {
                panic!(
                    "Fragment shader compilation failed: {}",
                    gl.get_shader_info_log(fragment_shader)
                );
            } else {
                println!("Fragment shader compiled successfully.");
            }

            let program = gl.create_program().unwrap();
            gl.attach_shader(program, vertex_shader);
            gl.attach_shader(program, fragment_shader);
            gl.link_program(program);
            if !gl.get_program_link_status(program) {
                panic!(
                    "Program linking failed: {}",
                    gl.get_program_info_log(program)
                );
            } else {
                println!("Program linked successfully");
            }

            let shader = Shader {
                program,
                u_view_proj_location: gl.get_uniform_location(program, "u_view_proj").unwrap(),
                u_camera_position_location: gl
                    .get_uniform_location(program, "u_camera_position")
                    .unwrap(),
                u_light_direction_location: gl
                    .get_uniform_location(program, "u_light_direction")
                    .unwrap(),
                u_light_color_location: gl.get_uniform_location(program, "u_light_color").unwrap(),
                u_albedo_location: gl.get_uniform_location(program, "u_albedo").unwrap(),
                u_normal_location: gl.get_uniform_location(program, "u_normal").unwrap(),
                u_roughness_location: gl.get_uniform_location(program, "u_roughness").unwrap(),
                u_base_reflectance_location: gl
                    .get_uniform_location(program, "u_base_reflectance")
                    .unwrap(),
                u_visualize_normals_location: gl
                    .get_uniform_location(program, "u_visualize_normals")
                    .unwrap(),
                u_visualize_edges_location: gl
                    .get_uniform_location(program, "u_visualize_edges")
                    .unwrap(),
                u_edge_thickness_location: gl
                    .get_uniform_location(program, "u_edge_thickness")
                    .unwrap(),
            };

            gl.delete_shader(vertex_shader);
            gl.delete_shader(fragment_shader);

            shader
        }
    }
}
