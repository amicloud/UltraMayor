#version 310 es

precision highp float;
precision highp int;

layout(location = 0) in vec3 position; // Vertex position
layout(location = 1) in vec3 normal;   // Vertex normal
layout(location = 2) in vec3 barycentric;   // Barycentric coordinates
layout(location = 3) in vec2 uv_albedo; // Albedo texture UV coordinates
layout(location = 4) in vec2 uv_normal; // Normal map texture UV coordinates

// Per-instance model matrix — declared as four vec4 attributes with fixed locations
layout(location = 5) in vec4 instance_model_col0;
layout(location = 6) in vec4 instance_model_col1;
layout(location = 7) in vec4 instance_model_col2;
layout(location = 8) in vec4 instance_model_col3;

uniform mat4 u_view_proj;  // View-projection matrix
uniform vec3 u_camera_position; // Camera position in world space

out vec3 v_normal;      // Normal in world space (interpolated in fragment shader)
out vec3 v_view_dir;    // View direction in world space (interpolated)
out vec3 v_barycentric;
out vec3 v_camera_position;
out vec2 v_uv_albedo;
out vec2 v_uv_normal;

void main() {
    mat4 instance_model = mat4(
        instance_model_col0,
        instance_model_col1,
        instance_model_col2,
        instance_model_col3
    );
    // Transform the vertex position to clip space
    gl_Position = u_view_proj * instance_model * vec4(position, 1.0);

    // Transform the normal into world space using the inverse transpose of the model matrix
    v_normal = mat3(transpose(inverse(instance_model))) * normal;

    // Calculate the view direction (camera to vertex) in world space
    vec3 world_pos = vec3(instance_model * vec4(position, 1.0)); // Position in world space
    v_view_dir = normalize(u_camera_position - world_pos); // View direction from camera to vertex

    // Pass barycentric coordinates to fragment shader
    v_barycentric = barycentric;

    // Pass camera position to fragment shader
    v_camera_position = u_camera_position;

    // Pass UV coordinates to fragment shader
    v_uv_albedo = uv_albedo;
    v_uv_normal = uv_normal;
}
