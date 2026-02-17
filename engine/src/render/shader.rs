use std::hash::Hash;
use std::{collections::HashMap, ffi::OsStr, fs};

use crate::handles::TextureHandle;
use glow::HasContext;

pub enum UniformValue {
    Float(f32),
    Vec3(glam::Vec3),
    Mat4(glam::Mat4),
    #[allow(dead_code)]
    Int(i32),
    Texture {
        handle: TextureHandle,
        unit: u32,
    },
}

#[derive(Debug, Clone, Copy)]
pub enum VertexAttribType {
    Float32,
    Vec2,
    Vec3,
    Vec4,
}

#[derive(Debug, Clone, Copy)]
pub enum InputRate {
    PerVertex,
    PerInstance,
}

pub struct ShaderAttrib {
    pub name: String,
    pub location: u32,
    pub ty: VertexAttribType,
    pub rate: InputRate, // PerVertex | PerInstance
}

pub struct Shader {
    pub program: glow::Program,
    pub uniforms: HashMap<String, glow::UniformLocation>,
    pub attributes: Vec<ShaderAttrib>,
}

impl Hash for Shader {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        // Hash based on the program ID
        self.program.hash(state);
    }
}

impl Shader {
    pub fn new(gl: &glow::Context, vertex_src: &OsStr, fragment_src: &OsStr) -> Self {
        unsafe {
            let vertex_shader = gl.create_shader(glow::VERTEX_SHADER).unwrap();
            let vertex_shader_source = fs::read_to_string(vertex_src.to_str().unwrap())
                .unwrap_or_else(|_| {
                    panic!(
                        "Failed to read vertex shader file {}",
                        vertex_src.to_str().unwrap()
                    )
                });

            gl.shader_source(vertex_shader, &vertex_shader_source);
            gl.compile_shader(vertex_shader);
            if !gl.get_shader_compile_status(vertex_shader) {
                panic!(
                    "Vertex shader compilation failed: {}",
                    gl.get_shader_info_log(vertex_shader)
                );
            }

            let fragment_shader = gl.create_shader(glow::FRAGMENT_SHADER).unwrap();
            let fragment_shader_source = fs::read_to_string(fragment_src.to_str().unwrap())
                .unwrap_or_else(|_| {
                    panic!(
                        "Failed to read fragment shader file {}",
                        fragment_src.to_str().unwrap()
                    )
                });
            gl.shader_source(fragment_shader, &fragment_shader_source);
            gl.compile_shader(fragment_shader);
            if !gl.get_shader_compile_status(fragment_shader) {
                panic!(
                    "Fragment shader compilation failed: {}",
                    gl.get_shader_info_log(fragment_shader)
                );
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
            }

            let mut uniforms = HashMap::new();
            let count = gl.get_program_parameter_i32(program, glow::ACTIVE_UNIFORMS);

            for i in 0..count {
                if let Some(info) = gl.get_active_uniform(program, i as u32)
                    && let Some(loc) = gl.get_uniform_location(program, &info.name)
                {
                    uniforms.insert(info.name, loc);
                }
            }

            let mut attributes = Vec::new();
            let attrib_count = gl.get_active_attributes(program) as i32;
            for i in 0..attrib_count {
                if let Some(info) = gl.get_active_attribute(program, i as u32) {
                    let raw_name = info.name;
                    let name = raw_name.trim_end_matches("[0]").to_string();
                    let Some(location) = gl.get_attrib_location(program, &name) else {
                        continue;
                    };

                    let rate = if name.starts_with("instance_") {
                        InputRate::PerInstance
                    } else {
                        InputRate::PerVertex
                    };

                    match info.atype {
                        glow::FLOAT => attributes.push(ShaderAttrib {
                            name: name.clone(),
                            location,
                            ty: VertexAttribType::Float32,
                            rate,
                        }),
                        glow::FLOAT_VEC2 => attributes.push(ShaderAttrib {
                            name: name.clone(),
                            location,
                            ty: VertexAttribType::Vec2,
                            rate,
                        }),
                        glow::FLOAT_VEC3 => attributes.push(ShaderAttrib {
                            name: name.clone(),
                            location,
                            ty: VertexAttribType::Vec3,
                            rate,
                        }),
                        glow::FLOAT_VEC4 => attributes.push(ShaderAttrib {
                            name: name.clone(),
                            location,
                            ty: VertexAttribType::Vec4,
                            rate,
                        }),
                        glow::FLOAT_MAT4 => {
                            for col in 0..4 {
                                attributes.push(ShaderAttrib {
                                    name: format!("{}_col{}", name, col),
                                    location: location + col,
                                    ty: VertexAttribType::Vec4,
                                    rate,
                                });
                            }
                        }
                        _ => {}
                    }
                }
            }

            let shader = Shader {
                program,
                uniforms,
                attributes,
            };

            gl.delete_shader(vertex_shader);
            gl.delete_shader(fragment_shader);

            shader
        }
    }
}
