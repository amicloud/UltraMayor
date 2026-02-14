// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use glam::{Mat4, Vec3};

use crate::collider_component::ConvexCollider;

const EPA_MAX_ITERATIONS: usize = 64;
const EPA_TOLERANCE: f32 = 1e-4;
const EPSILON: f32 = 1e-6;

#[derive(Debug, Clone, PartialEq)]
pub struct EpaResult {
    pub normal: Vec3,
    pub penetration_depth: f32,
}

#[derive(Debug, Clone)]
struct Face {
    indices: [usize; 3],
    normal: Vec3,
    distance: f32,
}
pub fn epa(
    a: &ConvexCollider,
    a_transform: Mat4,
    b: &ConvexCollider,
    b_transform: Mat4,
    simplex: &[Vec3],
) -> Option<EpaResult> {
    let (mut vertices, mut faces) =
        build_initial_polytope(a, a_transform, b, b_transform, simplex)?;

    vertices.reserve(64);
    faces.reserve(128);

    let mut horizon_edges: Vec<(usize, usize)> = Vec::with_capacity(64);
    let mut new_faces: Vec<Face> = Vec::with_capacity(128);

    // Keep track of previous closest face normal to reduce oscillation
    // This should probably be getting used!
    let prev_normal = Vec3::ZERO;

    for _ in 0..EPA_MAX_ITERATIONS {
        horizon_edges.clear();
        new_faces.clear();
        {
            let closest_index = find_closest_face(&faces)?;

            // Stabilize: pick the face whose normal is most aligned with previous frame
            let closest_face = if prev_normal != Vec3::ZERO {
                faces
                    .iter()
                    .min_by(|f1, f2| {
                        let d1 = f1.normal.dot(prev_normal).abs();
                        let d2 = f2.normal.dot(prev_normal).abs();
                        d2.partial_cmp(&d1).unwrap() // pick most aligned
                    })
                    .unwrap()
            } else {
                &faces[closest_index]
            };

            let support = support_minkowski(a, a_transform, b, b_transform, closest_face.normal);
            let support_distance = closest_face.normal.dot(support);
            let distance_delta = support_distance - closest_face.distance;

            if distance_delta <= EPA_TOLERANCE {
                let (normal, penetration_depth) = orient_result(
                    closest_face.normal,
                    closest_face.distance,
                    a_transform,
                    b_transform,
                );
                return Some(EpaResult {
                    normal,
                    penetration_depth,
                });
            }

            let new_index = vertices.len();
            vertices.push(support);

            // ---- Visibility pass ----
            for face in faces.iter() {
                if is_face_visible(&vertices, face, support) {
                    add_edge(&mut horizon_edges, face.indices[0], face.indices[1]);
                    add_edge(&mut horizon_edges, face.indices[1], face.indices[2]);
                    add_edge(&mut horizon_edges, face.indices[2], face.indices[0]);
                } else {
                    // Skip degenerate faces
                    if face.normal.length_squared() > EPSILON {
                        new_faces.push(face.clone());
                    }
                }
            }

            // SAFETY: if no horizon edges, break early
            if horizon_edges.is_empty() {
                break;
            }

            // ---- Rebuild faces from horizon ----
            for (a_idx, b_idx) in &horizon_edges {
                if let Some(face) = make_face_outward(&vertices, *a_idx, *b_idx, new_index) {
                    // Skip faces with nearly zero normal
                    if face.normal.length_squared() > EPSILON {
                        new_faces.push(face);
                    }
                }
            }
        }
        std::mem::swap(&mut faces, &mut new_faces);

        // Record previous normal for stability
        // prev_normal = closest_face.normal;
    }

    // Fallback: return closest face we have
    let closest_index = find_closest_face(&faces)?;
    let face = &faces[closest_index];
    let (normal, penetration_depth) =
        orient_result(face.normal, face.distance, a_transform, b_transform);

    Some(EpaResult {
        normal,
        penetration_depth,
    })
}
fn orient_result(
    mut normal: Vec3,
    penetration_depth: f32,
    a_transform: Mat4,
    b_transform: Mat4,
) -> (Vec3, f32) {
    let a_center = a_transform.transform_point3(Vec3::ZERO);
    let b_center = b_transform.transform_point3(Vec3::ZERO);
    let ab = b_center - a_center;

    if normal.dot(ab) < 0.0 {
        normal = -normal;
    }

    (normal, penetration_depth)
}

fn build_initial_polytope(
    a: &ConvexCollider,
    a_transform: Mat4,
    b: &ConvexCollider,
    b_transform: Mat4,
    simplex: &[Vec3],
) -> Option<(Vec<Vec3>, Vec<Face>)> {
    let mut vertices: Vec<Vec3> = simplex.to_vec();

    match vertices.len() {
        4 => {}
        3 => {
            let dir = direction_to_origin_triangle(vertices[0], vertices[1], vertices[2])
                .unwrap_or_else(|| fallback_direction(vertices[0], vertices[1]));
            let d = support_unique(a, a_transform, b, b_transform, dir, &vertices)?;
            vertices.push(d);
        }
        2 => {
            let dir = direction_to_origin_line(vertices[0], vertices[1])
                .unwrap_or_else(|| fallback_direction(vertices[0], vertices[1]));
            let c = support_unique(a, a_transform, b, b_transform, dir, &vertices)?;
            vertices.push(c);

            let dir = direction_to_origin_triangle(vertices[0], vertices[1], vertices[2])
                .unwrap_or_else(|| fallback_direction(vertices[0], vertices[1]));
            let d = support_unique(a, a_transform, b, b_transform, dir, &vertices)?;
            vertices.push(d);
        }
        _ => return None,
    }

    if vertices.len() != 4 {
        return None;
    }

    let mut faces = Vec::with_capacity(4);
    let face_indices = [(0, 1, 2), (0, 3, 1), (0, 2, 3), (1, 3, 2)];

    for (i0, i1, i2) in face_indices {
        if let Some(face) = make_face_outward(&vertices, i0, i1, i2) {
            faces.push(face);
        }
    }

    if faces.len() < 4 {
        return None;
    }

    Some((vertices, faces))
}

fn fallback_direction(a: Vec3, b: Vec3) -> Vec3 {
    let ab = b - a;
    let axis = if ab.cross(Vec3::X).length_squared() > EPSILON {
        Vec3::X
    } else if ab.cross(Vec3::Y).length_squared() > EPSILON {
        Vec3::Y
    } else {
        Vec3::Z
    };

    let perp = ab.cross(axis);
    if perp.length_squared() <= EPSILON {
        Vec3::X
    } else {
        perp.normalize()
    }
}

fn direction_to_origin_line(a: Vec3, b: Vec3) -> Option<Vec3> {
    let ab = b - a;
    let ao = -a;
    let perp = ab.cross(ao).cross(ab);
    if perp.length_squared() <= EPSILON {
        None
    } else {
        Some(perp.normalize())
    }
}

fn direction_to_origin_triangle(a: Vec3, b: Vec3, c: Vec3) -> Option<Vec3> {
    let ab = b - a;
    let ac = c - a;
    let mut normal = ab.cross(ac);
    if normal.length_squared() <= EPSILON {
        return None;
    }

    if normal.dot(-a) < 0.0 {
        normal = -normal;
    }

    Some(normal.normalize())
}

fn support_unique(
    a: &ConvexCollider,
    a_transform: Mat4,
    b: &ConvexCollider,
    b_transform: Mat4,
    dir: Vec3,
    existing: &[Vec3],
) -> Option<Vec3> {
    let mut support = support_minkowski(a, a_transform, b, b_transform, dir);
    if existing
        .iter()
        .any(|p| (*p - support).length_squared() <= EPSILON)
    {
        support = support_minkowski(a, a_transform, b, b_transform, -dir);
        if existing
            .iter()
            .any(|p| (*p - support).length_squared() <= EPSILON)
        {
            return None;
        }
    }

    Some(support)
}

fn make_face_outward(vertices: &[Vec3], a: usize, b: usize, c: usize) -> Option<Face> {
    let ab = vertices[b] - vertices[a];
    let ac = vertices[c] - vertices[a];
    let mut normal = ab.cross(ac);

    // Degenerate or near-degenerate triangle
    if normal.length_squared() < EPSILON {
        return None;
    }

    normal = normal.normalize();
    let mut distance = normal.dot(vertices[a]);

    if distance < 0.0 {
        normal = -normal;
        distance = -distance;
    }

    Some(Face {
        indices: [a, b, c],
        normal,
        distance,
    })
}

fn find_closest_face(faces: &[Face]) -> Option<usize> {
    let mut best_index = None;
    let mut best_distance = f32::INFINITY;

    for (i, face) in faces.iter().enumerate() {
        if face.distance < best_distance {
            best_distance = face.distance;
            best_index = Some(i);
        }
    }

    best_index
}

fn is_face_visible(vertices: &[Vec3], face: &Face, point: Vec3) -> bool {
    let face_point = vertices[face.indices[0]];
    face.normal.dot(point - face_point) > 0.0
}

fn add_edge(edges: &mut Vec<(usize, usize)>, a: usize, b: usize) {
    if let Some(index) = edges.iter().position(|(u, v)| *u == b && *v == a) {
        edges.swap_remove(index);
    } else {
        edges.push((a, b));
    }
}

fn support_minkowski(
    a: &ConvexCollider,
    a_transform: Mat4,
    b: &ConvexCollider,
    b_transform: Mat4,
    dir: Vec3,
) -> Vec3 {
    let p1 = a.support(a_transform, dir);
    let p2 = b.support(b_transform, -dir);
    p1 - p2
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collider_component::{CollisionLayer, ConvexCollider};
    use crate::gjk::{GjkResult, gjk_intersect};
    use crate::mesh::AABB;
    use crate::transform_component::TransformComponent;
    use glam::{Quat, Vec3};

    fn transform_at(position: Vec3, rotation: Quat) -> Mat4 {
        TransformComponent {
            position,
            rotation,
            scale: Vec3::ONE,
        }
        .to_mat4()
    }

    fn run_epa(
        a: &ConvexCollider,
        a_transform: Mat4,
        b: &ConvexCollider,
        b_transform: Mat4,
    ) -> EpaResult {
        let result = gjk_intersect(a, a_transform, b, b_transform);
        let simplex = match result {
            GjkResult::Intersection(hit) => hit.simplex,
            GjkResult::NoIntersection => panic!("Expected intersection."),
        };

        epa(a, a_transform, b, b_transform, &simplex).expect("EPA failed")
    }

    fn assert_normal_points_from_a_to_b(normal: Vec3, a_transform: Mat4, b_transform: Mat4) {
        let a_center = a_transform.transform_point3(Vec3::ZERO);
        let b_center = b_transform.transform_point3(Vec3::ZERO);
        let ab = b_center - a_center;
        if ab.length_squared() > EPSILON {
            assert!(normal.dot(ab) > 0.0);
        }
    }

    #[test]
    fn epa_box_vs_box_axis_aligned() {
        let a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let b = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let b_transform = transform_at(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY);

        let result = run_epa(&a, a_transform, &b, b_transform);
        assert!(result.penetration_depth > 0.0);
        assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
    }

    #[test]
    fn epa_box_vs_box_rotated() {
        let a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let b = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let rotation = Quat::from_rotation_z(0.5);
        let b_transform = transform_at(Vec3::new(0.8, 0.2, 0.0), rotation);

        let result = run_epa(&a, a_transform, &b, b_transform);
        assert!(result.penetration_depth > 0.0);
        assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
    }

    #[test]
    fn epa_sphere_vs_box() {
        let sphere = ConvexCollider::sphere(2.0, CollisionLayer::Default);
        let box_collider = ConvexCollider::cuboid_from_aabb(
            AABB {
                min: Vec3::splat(-1.0),
                max: Vec3::splat(1.0),
            },
            CollisionLayer::Default,
        );

        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let b_transform = transform_at(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY);

        let result = run_epa(&sphere, a_transform, &box_collider, b_transform);
        assert!(result.penetration_depth > 0.0);
        assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
    }

    #[test]
    fn epa_deep_penetration() {
        let a = ConvexCollider::cube(4.0, CollisionLayer::Default);
        let b = ConvexCollider::cube(4.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let b_transform = transform_at(Vec3::new(0.1, 0.1, 0.0), Quat::IDENTITY);

        let result = run_epa(&a, a_transform, &b, b_transform);
        assert!(result.penetration_depth > 0.0);
        assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
    }

    #[test]
    fn epa_nearly_touching() {
        let a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let b = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let b_transform = transform_at(Vec3::new(1.99, 0.0, 0.0), Quat::IDENTITY);

        let result = run_epa(&a, a_transform, &b, b_transform);
        assert!(result.penetration_depth > 0.0);
        assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
    }
}
