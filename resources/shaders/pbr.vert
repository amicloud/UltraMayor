#version 310 es

precision highp float;
precision highp int;

layout(location = 0) in vec3 position; // Vertex position
layout(location = 1) in vec3 normal;   // Vertex normal
layout(location = 2) in vec3 barycentric;   // Barycentric coordinates

uniform mat4 view_proj;  // View-projection matrix
uniform mat4 model;      // Model matrix
uniform vec3 camera_position; // Camera position in world space

out vec3 v_normal;      // Normal in world space (interpolated in fragment shader)
out vec3 v_view_dir;    // View direction in world space (interpolated)
out vec3 v_barycentric;
out float v_distance_from_camera;
out vec3 v_camera_position;

void main() {
    // Transform the vertex position to clip space
    gl_Position = view_proj * model * vec4(position, 1.0);

    // Transform the normal into world space using the inverse transpose of the model matrix
    v_normal = mat3(transpose(inverse(model))) * normal;

    // Calculate the view direction (camera to vertex) in world space
    vec3 world_pos = vec3(model * vec4(position, 1.0)); // Position in world space
    v_view_dir = normalize(camera_position - world_pos); // View direction from camera to vertex

    // Pass barycentric coordinates to fragment shader
    v_barycentric = barycentric;

    // Pass camera position to fragment shader
    v_camera_position = camera_position;
}
