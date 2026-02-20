// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use glam::{Mat4, Vec3};

use crate::{components::collider_component::ConvexCollider, physics::{self, gjk::support_minkowski}};
use physics::physics_resource::ContactManifold;

const EPA_MAX_ITERATIONS: usize = 64;
const EPA_TOLERANCE: f32 = 1e-4;
const EPSILON: f32 = 1e-6;
const FACE_VISIBILITY_EPSILON: f32 = 1e-6;

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
    previous_manifold: Option<&ContactManifold>,
) -> Option<EpaResult> {
    let (mut vertices, mut faces) = build_initial_polytope(simplex)?;
    let manifold_bias = manifold_bias_direction(previous_manifold);

    vertices.reserve(64);
    faces.reserve(128);

    let mut horizon_edges: Vec<(usize, usize)> = Vec::with_capacity(64);
    let mut new_faces: Vec<Face> = Vec::with_capacity(128);

    if let Some(bias) = manifold_bias {
        for dir in [bias, -bias] {
            let support = support_minkowski(a, a_transform, b, b_transform, dir);
            let _ = expand_polytope(
                &mut vertices,
                &mut faces,
                support,
                &mut horizon_edges,
                &mut new_faces,
            );
        }
    }

    for _ in 0..EPA_MAX_ITERATIONS {
        let closest_index = find_closest_face(&faces)?;
        let closest_face = &faces[closest_index];
        let closest_normal = closest_face.normal;
        let closest_distance = closest_face.distance;

        let search_dir = blend_search_direction(closest_normal, manifold_bias);
        let support = support_minkowski(a, a_transform, b, b_transform, search_dir);
        let support_distance = closest_normal.dot(support);
        let distance_delta = support_distance - closest_distance;

        if distance_delta <= EPA_TOLERANCE {
            let (normal, penetration_depth) =
                orient_result(closest_normal, closest_distance, a_transform, b_transform);
            return Some(EpaResult {
                normal,
                penetration_depth,
            });
        }

        if !expand_polytope(
            &mut vertices,
            &mut faces,
            support,
            &mut horizon_edges,
            &mut new_faces,
        ) {
            let (normal, penetration_depth) =
                orient_result(closest_normal, closest_distance, a_transform, b_transform);
            return Some(EpaResult {
                normal,
                penetration_depth,
            });
        }
    }
    // Fallback: return closest face we have
    let closest_index = find_closest_face(&faces)?;
    let face = &faces[closest_index];
    let (normal, penetration_depth) =
        orient_result(face.normal, face.distance, a_transform, b_transform);

    Some(EpaResult {
        normal, // Flip normal to point from A to B
        penetration_depth,
    })
}

fn manifold_bias_direction(previous_manifold: Option<&ContactManifold>) -> Option<Vec3> {
    let normal = previous_manifold?.normal;
    if normal.length_squared() <= EPSILON {
        return None;
    }
    Some(normal.normalize())
}

fn blend_search_direction(face_normal: Vec3, manifold_bias: Option<Vec3>) -> Vec3 {
    let Some(bias) = manifold_bias else {
        return face_normal;
    };

    let alignment = face_normal.dot(bias);
    if alignment <= 0.0 {
        return face_normal;
    }

    let weight = (0.25 * alignment).clamp(0.0, 0.25);
    let blended = face_normal * (1.0 - weight) + bias * weight;
    if blended.length_squared() <= EPSILON {
        face_normal
    } else {
        blended.normalize()
    }
}

fn expand_polytope(
    vertices: &mut Vec<Vec3>,
    faces: &mut Vec<Face>,
    support: Vec3,
    horizon_edges: &mut Vec<(usize, usize)>,
    new_faces: &mut Vec<Face>,
) -> bool {
    if vertices
        .iter()
        .any(|v| (*v - support).length_squared() <= EPSILON * 10.0)
    {
        return false;
    }

    let new_index = vertices.len();
    vertices.push(support);

    horizon_edges.clear();
    new_faces.clear();

    for face in faces.iter() {
        if is_face_visible(vertices, face, support) {
            add_edge(horizon_edges, face.indices[0], face.indices[1]);
            add_edge(horizon_edges, face.indices[1], face.indices[2]);
            add_edge(horizon_edges, face.indices[2], face.indices[0]);
        } else if face.normal.length_squared() > EPSILON {
            new_faces.push(face.clone());
        }
    }

    if horizon_edges.is_empty() {
        let _ = vertices.pop();
        new_faces.clear();
        return false;
    }

    for (a_idx, b_idx) in horizon_edges.iter() {
        if let Some(face) = make_face_outward(vertices, *a_idx, *b_idx, new_index)
            && face.normal.length_squared() > EPSILON
        {
            new_faces.push(face);
        }
    }

    if new_faces.is_empty() {
        let _ = vertices.pop();
        return false;
    }

    std::mem::swap(faces, new_faces);
    true
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

fn build_initial_polytope(simplex: &[Vec3]) -> Option<(Vec<Vec3>, Vec<Face>)> {
    let vertices: Vec<Vec3> = simplex.to_vec();

    match vertices.len() {
        4 => {}
        _ => {
            println!(
                "EPA requires a tetrahedral simplex, got {} vertices",
                vertices.len()
            );
            return None;
        }
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
    face.normal.dot(point - face_point) > FACE_VISIBILITY_EPSILON
}

fn add_edge(edges: &mut Vec<(usize, usize)>, a: usize, b: usize) {
    if let Some(index) = edges.iter().position(|(u, v)| *u == b && *v == a) {
        edges.swap_remove(index);
    } else {
        edges.push((a, b));
    }
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;
    use glam::{Quat, Vec3};

    use crate::assets::mesh::Aabb;
    use crate::components::collider_component::{CollisionLayer, ConvexCollider};
    use crate::components::transform_component::TransformComponent;

    use physics::gjk::{GjkResult, gjk_intersect};

    use super::*;
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

        epa(a, a_transform, b, b_transform, &simplex, None).expect("EPA failed")
    }

    fn assert_normal_points_from_a_to_b(normal: Vec3, a_transform: Mat4, b_transform: Mat4) {
        let a_center = a_transform.transform_point3(Vec3::ZERO);
        let b_center = b_transform.transform_point3(Vec3::ZERO);
        let ab = b_center - a_center;
        if ab.length_squared() > EPSILON {
            // In symmetric/tie cases, the separating axis can be orthogonal to center-to-center.
            assert!(normal.dot(ab) >= -1e-4);
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
            Aabb {
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

    #[test]
    fn epa_non_uniform_cuboid_axis_aligned() {
        // Non-uniform cuboid: 4×2×2 box
        let a = ConvexCollider::cuboid(Vec3::new(4.0, 2.0, 2.0), CollisionLayer::Default);
        let b = ConvexCollider::cuboid(Vec3::new(4.0, 2.0, 2.0), CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let b_transform = transform_at(Vec3::new(3.0, 0.0, 0.0), Quat::IDENTITY);

        let result = run_epa(&a, a_transform, &b, b_transform);
        // Overlap on X: (2 + 2) - 3 = 1.0
        assert!(result.penetration_depth > 0.0);
        assert!((result.penetration_depth - 1.0).abs() < 0.1);
        assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
        // Normal should be approximately +X
        assert!(result.normal.x.abs() > 0.9);
    }

    #[test]
    fn epa_non_uniform_cuboid_rotated() {
        let a = ConvexCollider::cuboid(Vec3::new(6.0, 1.0, 1.0), CollisionLayer::Default);
        let b = ConvexCollider::cuboid(Vec3::new(6.0, 1.0, 1.0), CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let b_transform = transform_at(
            Vec3::new(0.0, 1.0, 0.0),
            Quat::from_rotation_z(90.0_f32.to_radians()),
        );

        let result = run_epa(&a, a_transform, &b, b_transform);
        assert!(result.penetration_depth > 0.0);
        assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
    }

    #[test]
    fn epa_cuboid_vs_sphere() {
        let cuboid = ConvexCollider::cuboid(Vec3::new(2.0, 4.0, 2.0), CollisionLayer::Default);
        let sphere = ConvexCollider::sphere(1.5, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let b_transform = transform_at(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY);

        let result = run_epa(&cuboid, a_transform, &sphere, b_transform);
        // Cuboid y half-extent = 2.0, sphere radius = 1.5, distance = 2.0
        // Overlap = 2.0 + 1.5 - 2.0 = 1.5
        assert!(result.penetration_depth > 0.0);
        assert!((result.penetration_depth - 1.5).abs() < 0.4);
        assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
    }

    #[test]
    fn epa_box_vs_box_colliding_nearly_coplanar_1e() {
        let a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let b = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);

        let e = EPSILON * 1.0;

        let b_transform = transform_at(Vec3::new(0.0, 0.0, 2.0 - e), Quat::IDENTITY);

        let result = run_epa(&a, a_transform, &b, b_transform);
        assert!(result.penetration_depth > 0.0);
        assert_approx_eq!(result.penetration_depth, e, EPSILON);
        assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
    }

    #[test]
    fn epa_box_vs_box_colliding_nearly_coplanar_sweep() {
        let a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let b = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let coplanar_point = 2.0;

        for i in 1..10_000 {
            let e = (1000.0 * EPSILON) / i as f32;
            let b_transform =
                transform_at(Vec3::new(0.0, 0.0, coplanar_point - (e)), Quat::IDENTITY);
            let result = run_epa(&a, a_transform, &b, b_transform);
            assert!(result.penetration_depth > 0.0);
            assert_approx_eq!(result.penetration_depth, e, EPSILON);
            assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
        }
    }

    #[test]
    fn epa_box_vs_prism_colliding_nearly_coplanar_sweep() {
        // let a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a = ConvexCollider::triangle_prism(
            Vec3::new(-5.0, 5.0, 0.0),
            Vec3::new(5.0, 5.0, 0.0),
            Vec3::new(0.0, -5.0, 0.0),
            1.0,
            CollisionLayer::Default,
        );
        let b = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO, Quat::IDENTITY);
        let coplanar_point = 2.0;

        for i in 1..10_000 {
            let e = (1000.0 * EPSILON) / i as f32;
            let b_transform =
                transform_at(Vec3::new(0.0, 0.0, coplanar_point - (e)), Quat::IDENTITY);
            let result = run_epa(&a, a_transform, &b, b_transform);
            assert!(result.penetration_depth > 0.0);
            assert_approx_eq!(result.penetration_depth, e, EPSILON);
            assert_normal_points_from_a_to_b(result.normal, a_transform, b_transform);
        }
    }
}
