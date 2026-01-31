#version 330 core

in vec3 v_normal;
in vec3 v_view_dir;
in vec3 v_barycentric;
in vec3 v_camera_position;
in vec2 v_uv_albedo;
in vec2 v_uv_normal;
in mat3 v_tbn;

out vec4 fragColor;

uniform sampler2D u_albedo;
uniform sampler2D u_normal;
uniform vec3 u_light_direction;
uniform vec3 u_light_color;
uniform float u_roughness;
uniform vec3 u_base_reflectance;

const float PI = 3.14159265359;

// -------------------- Microfacet helpers --------------------

float D(float alpha, vec3 N, vec3 H) {
    float a2 = alpha * alpha;
    float NdotH = max(dot(N, H), 0.0);
    float denom = (NdotH * NdotH) * (a2 - 1.0) + 1.0;
    return a2 / (PI * denom * denom);
}

float G1(float alpha, vec3 N, vec3 X) {
    float NdotX = max(dot(N, X), 0.0);
    float k = (alpha + 1.0) * (alpha + 1.0) / 8.0;
    return NdotX / (NdotX * (1.0 - k) + k + 1e-5);
}

float G(float alpha, vec3 N, vec3 V, vec3 L) {
    return G1(alpha, N, V) * G1(alpha, N, L);
}

vec3 F(vec3 F0, vec3 V, vec3 H) {
    float VdotH = max(dot(V, H), 0.0);
    return F0 + (vec3(1.0) - F0) * pow(1.0 - VdotH, 5.0);
}

// -------------------- Main --------------------

void main() {
    // Albedo
    vec3 albedo = texture(u_albedo, v_uv_albedo).rgb;

    // Normal mapping (tangent → world)
    vec3 N_tangent = texture(u_normal, v_uv_normal).xyz * 2.0 - 1.0;
    N_tangent.xy *= 2.0; // Increase normal map strength
    vec3 N = normalize(v_tbn * N_tangent);

    vec3 V = normalize(v_view_dir);
    vec3 L = normalize(u_light_direction);
    vec3 H = normalize(V + L);

    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 0.0);

    float alpha = u_roughness * u_roughness;
    vec3 F0 = u_base_reflectance;

    // Specular BRDF
    vec3 F_spec = F(F0, V, H);
    float G_spec = G(alpha, N, V, L);
    float D_spec = D(alpha, N, H);
    vec3 specular = (F_spec * G_spec * D_spec)
        / (4.0 * NdotL * NdotV + 1e-6);

    // Diffuse (energy-conserving-ish)
    vec3 diffuse = albedo * (1.0 - F_spec);

    // Direct lighting
    vec3 direct_light =
        (diffuse + specular) * u_light_color * NdotL;

    // ✅ Albedo-preserving ambient
    vec3 ambient = albedo * u_light_color * 0.2;

    vec3 color = direct_light + ambient;

    fragColor = vec4(color, 1.0);
}