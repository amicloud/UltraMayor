// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.
#version 310 es

precision highp float;
precision highp int;
// Removed precision highp uint;

layout(local_size_x = 256) in;

// Binding points must match the SSBO bindings in Rust
layout(std430, binding = 0) buffer MeshBuffer {
    float vertices[]; // Flattened list of triangle vertices
};

layout(std430, binding = 1) buffer SlicePlanes {
    float slice_z[]; // List of slice plane z-values
};

layout(std430, binding = 2) buffer OutputSegments {
    float segments[]; // Output segments as (x1, y1, x2, y2, slice_index)
};

// Atomic counter for output segments
layout(binding = 3, offset = 0) uniform atomic_uint segment_count;

void main() {
    // Cast gl_GlobalInvocationID.x to int
    int idx = int(gl_GlobalInvocationID.x);

    // Each triangle has 3 vertices (x, y, z)
    int tri_idx = idx * 9; // 3 vertices * 3 components

    // Compare int with int
    if (tri_idx + 8 >= vertices.length())
        return; // Out of bounds

    // Read triangle vertices using int indices
    vec3 v0 = vec3(vertices[tri_idx], vertices[tri_idx + 1], vertices[tri_idx + 2]);
    vec3 v1 = vec3(vertices[tri_idx + 3], vertices[tri_idx + 4], vertices[tri_idx + 5]);
    vec3 v2 = vec3(vertices[tri_idx + 6], vertices[tri_idx + 7], vertices[tri_idx + 8]);

    // Iterate over slice planes using int
    for(int s = 0; s < slice_z.length(); s++) {
        float z = slice_z[s];

        // Compute distances from vertices to the plane
        float d0 = v0.z - z;
        float d1 = v1.z - z;
        float d2 = v2.z - z;

        bool all_positive = (d0 > 0.0) && (d1 > 0.0) && (d2 > 0.0);
        bool all_negative = (d0 < 0.0) && (d1 < 0.0) && (d2 < 0.0);

        if (all_positive || all_negative)
            continue; // Triangle does not intersect the plane

        // Find intersection points
        vec3 p[2];
        int count = 0;

        float epsilon = 1e-6;

        // Edge v0-v1
        if (((d0 > epsilon && d1 < -epsilon) || (d0 < -epsilon && d1 > epsilon)) ||
            (abs(d0) < epsilon && abs(d1) > epsilon) || (abs(d0) > epsilon && abs(d1) < epsilon)) {
            float denom = d0 - d1;
            if (abs(denom) > epsilon) {
                float t = d0 / denom;
                p[count++] = mix(v0, v1, t);
            } else {
                // Handle potential division by zero
                p[count++] = (v0 + v1) * 0.5;
            }
        }

        // Edge v1-v2
        if (((d1 > epsilon && d2 < -epsilon) || (d1 < -epsilon && d2 > epsilon)) ||
            (abs(d1) < epsilon && abs(d2) > epsilon) || (abs(d1) > epsilon && abs(d2) < epsilon)) {
            float denom = d1 - d2;
            if (abs(denom) > epsilon) {
                float t = d1 / denom;
                p[count++] = mix(v1, v2, t);
            } else {
                // Handle potential division by zero
                p[count++] = (v1 + v2) * 0.5;
            }
        }

        // Edge v2-v0
        if (((d2 > epsilon && d0 < -epsilon) || (d2 < -epsilon && d0 > epsilon)) ||
            (abs(d2) < epsilon && abs(d0) > epsilon) || (abs(d2) > epsilon && abs(d0) < epsilon)) {
            float denom = d2 - d0;
            if (abs(denom) > epsilon) {
                float t = d2 / denom;
                p[count++] = mix(v2, v0, t);
            } else {
                // Handle potential division by zero
                p[count++] = (v2 + v0) * 0.5;
            }
        }

        if(count == 2) {
            // Project to 2D (x, y)
            vec2 proj1 = p[0].xy;
            vec2 proj2 = p[1].xy;

            // Atomically get the current segment count
            uint current = atomicCounterIncrement(segment_count);

            // Convert current to int for indexing
            int out_idx = int(current) * 5; // Each segment has 5 floats (x1, y1, x2, y2, slice_index)

            // Compare int with int
            if(out_idx + 4 < segments.length()) {
                // Write the segment to the output buffer using int indices
                segments[out_idx] = proj1.x;
                segments[out_idx + 1] = proj1.y;
                segments[out_idx + 2] = proj2.x;
                segments[out_idx + 3] = proj2.y;
                segments[out_idx + 4] = float(s); // Store slice index as float
            }
        }
    }
}
