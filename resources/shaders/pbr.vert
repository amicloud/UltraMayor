#version 310 es
precision highp float;
precision highp int;

// Vertex attributes
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 barycentric;
layout(location = 3) in vec2 uv_albedo;
layout(location = 4) in vec2 uv_normal;
layout(location = 5) in vec4 tangent;

// Per-instance model matrix
layout(location = 6) in vec4 instance_model_col0;
layout(location = 7) in vec4 instance_model_col1;
layout(location = 8) in vec4 instance_model_col2;
layout(location = 9) in vec4 instance_model_col3;

uniform mat4 u_view_proj;
uniform vec3 u_camera_position;

out vec3 v_normal;
out vec3 v_view_dir;
out vec3 v_barycentric;
out vec3 v_camera_position;
out vec2 v_uv_albedo;
out vec2 v_uv_normal;
out mat3 v_tbn;

void main() {
    // Construct model matrix
    mat4 instance_model = mat4(
        instance_model_col0,
        instance_model_col1,
        instance_model_col2,
        instance_model_col3
    );

    // World position
    vec3 world_pos = vec3(instance_model * vec4(position, 1.0));
    gl_Position = u_view_proj * vec4(world_pos, 1.0);

    // Normal matrix
    mat3 normal_matrix = mat3(transpose(inverse(instance_model)));

    // World-space normal
    vec3 N = normalize(normal_matrix * normal);

    // World-space tangent (with Gram–Schmidt)
    vec3 T = normalize(normal_matrix * tangent.xyz);
    T = normalize(T - N * dot(N, T));

    // Reconstructed bitangent (glTF: tangent.w = handedness)
    vec3 B = tangent.w * normalize(cross(N, T));

    // TBN matrix
    v_tbn = mat3(
        normalize(T),
        normalize(B),
        normalize(N)
    );

    // Other outputs
    v_normal = N;
    v_view_dir = normalize(u_camera_position - world_pos);
    v_barycentric = barycentric;
    v_camera_position = u_camera_position;
    v_uv_albedo = uv_albedo;
    v_uv_normal = uv_normal;
}