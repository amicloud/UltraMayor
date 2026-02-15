// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use glam::{Mat4, Vec3};

use crate::collider_component::ConvexCollider;

const DEFAULT_MAX_ITERATIONS: usize = 32;
const EPSILON: f32 = 1e-6;

#[derive(Debug, Clone, PartialEq)]
pub struct GjkHit {
    /// Simplex points for optional EPA expansion, always contains 4 points for a tetrahedron
    pub simplex: Vec<Vec3>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum GjkResult {
    NoIntersection,
    Intersection(GjkHit),
}

/// Performs GJK intersection testing between two convex colliders.
/// Returns a tetrahedron suitable as EPA seed.
pub fn gjk_intersect(
    a: &ConvexCollider,
    a_transform: Mat4,
    b: &ConvexCollider,
    b_transform: Mat4,
) -> GjkResult {
    gjk_intersect_with_params(a, a_transform, b, b_transform, DEFAULT_MAX_ITERATIONS)
}

pub fn gjk_intersect_with_params(
    a: &ConvexCollider,
    a_transform: Mat4,
    b: &ConvexCollider,
    b_transform: Mat4,
    max_iterations: usize,
) -> GjkResult {
    let mut dir = initial_direction(a_transform, b_transform);
    let mut simplex: Vec<Vec3> = Vec::with_capacity(4);

    let support = support_minkowski(a, a_transform, b, b_transform, dir);
    simplex.push(support);
    dir = -support;

    for _ in 0..max_iterations {
        if dir.length_squared() <= EPSILON {
            return GjkResult::Intersection(GjkHit { simplex });
        }

        let support = support_minkowski(a, a_transform, b, b_transform, dir);
        if support.dot(dir) <= 0.0 {
            return GjkResult::NoIntersection;
        }

        simplex.push(support);
        if handle_simplex(&mut simplex, &mut dir) {
            return GjkResult::Intersection(GjkHit { simplex });
        }
    }

    GjkResult::NoIntersection
}

fn initial_direction(a_transform: Mat4, b_transform: Mat4) -> Vec3 {
    let a_center = a_transform.transform_point3(Vec3::ZERO);
    let b_center = b_transform.transform_point3(Vec3::ZERO);
    let dir = b_center - a_center;
    if dir.length_squared() <= EPSILON {
        Vec3::NEG_Z
    } else {
        dir
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

fn handle_simplex(simplex: &mut Vec<Vec3>, dir: &mut Vec3) -> bool {
    match simplex.len() {
        2 => handle_line(simplex, dir),
        3 => handle_triangle(simplex, dir),
        4 => handle_tetrahedron(simplex, dir),
        _ => false,
    }
}

fn handle_line(simplex: &mut Vec<Vec3>, dir: &mut Vec3) -> bool {
    let a = simplex[1];
    let b = simplex[0];
    let ab = b - a;
    let ab_len_sq = ab.length_squared();
    if ab_len_sq <= EPSILON {
        // Degenerate line (both points coincide). Keep both points so EPA
        // always receives at least a 2-point simplex.
        *dir = -a;
        // Let the main loop's zero-direction check handle containment.
        return false;
    }

    let t = (-a).dot(ab) / ab_len_sq;
    if t <= 0.0 {
        // Origin is closest to A. Keep A only but never declare containment
        // from a 1-point simplex—return false so the main loop adds another
        // support point.
        simplex.clear();
        simplex.push(a);
        *dir = -a;
        return false;
    }
    if t >= 1.0 {
        simplex.clear();
        simplex.push(b);
        *dir = -b;
        return false;
    }

    // Origin projects onto the interior of the line segment.
    let closest = a + ab * t;
    *dir = -closest;
    if dir.length_squared() <= EPSILON {
        // Origin is on the line segment. Don't declare containment yet;
        // keep the 2-point simplex and let the main loop detect it.
        *dir = Vec3::ZERO;
    }
    false
}

fn handle_triangle(simplex: &mut Vec<Vec3>, dir: &mut Vec3) -> bool {
    let a = simplex[2];
    let b = simplex[1];
    let c = simplex[0];

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let d1 = ab.dot(ao);
    let d2 = ac.dot(ao);
    if d1 <= 0.0 && d2 <= 0.0 {
        // Origin is closest to vertex A. Reduce to point but never declare
        // containment from a sub-dimensional simplex.
        simplex.clear();
        simplex.push(a);
        *dir = -a;
        return false;
    }

    let bo = -b;
    let d3 = ab.dot(bo);
    let d4 = ac.dot(bo);
    if d3 >= 0.0 && d4 <= d3 {
        simplex.clear();
        simplex.push(b);
        *dir = -b;
        return false;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let closest = a + ab * v;
        simplex.clear();
        simplex.push(b);
        simplex.push(a);
        *dir = -closest;
        return false;
    }

    let co = -c;
    let d5 = ab.dot(co);
    let d6 = ac.dot(co);
    if d6 >= 0.0 && d5 <= d6 {
        simplex.clear();
        simplex.push(c);
        *dir = -c;
        return false;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let closest = a + ac * w;
        simplex.clear();
        simplex.push(c);
        simplex.push(a);
        *dir = -closest;
        return false;
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let closest = b + (c - b) * w;
        simplex.clear();
        simplex.push(c);
        simplex.push(b);
        *dir = -closest;
        return false;
    }

    // Origin is inside the triangle (in barycentric sense). The search
    // direction is the triangle normal toward the origin.
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let closest = a + ab * v + ac * w;
    *dir = -closest;
    if dir.length_squared() <= EPSILON {
        // Origin is on the triangle plane. Keep the full 3-point simplex and
        // let the main loop's zero-direction check handle containment.
        *dir = Vec3::ZERO;
    }
    false
}

fn handle_tetrahedron(simplex: &mut Vec<Vec3>, dir: &mut Vec3) -> bool {
    let a = simplex[3];
    let b = simplex[2];
    let c = simplex[1];
    let d = simplex[0];
    let ao = -a;

    if let Some((face, normal)) = face_outside(a, b, c, d, ao) {
        *simplex = face;
        *dir = normal;
        return false;
    }

    if let Some((face, normal)) = face_outside(a, c, d, b, ao) {
        *simplex = face;
        *dir = normal;
        return false;
    }

    if let Some((face, normal)) = face_outside(a, d, b, c, ao) {
        *simplex = face;
        *dir = normal;
        return false;
    }

    true
}

fn face_outside(a: Vec3, b: Vec3, c: Vec3, opposite: Vec3, ao: Vec3) -> Option<(Vec<Vec3>, Vec3)> {
    let mut normal = (b - a).cross(c - a);
    if normal.dot(opposite - a) > 0.0 {
        normal = -normal;
    }

    if normal.dot(ao) > 0.0 {
        let face = vec![c, b, a];
        return Some((face, normal));
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collider_component::CollisionLayer;
    use crate::mesh::AABB;
    use crate::transform_component::TransformComponent;
    use glam::{Quat, Vec3};

    fn transform_at(position: Vec3) -> Mat4 {
        TransformComponent {
            position,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        }
        .to_mat4()
    }

    fn transform_at_with_rotation(position: Vec3, rotation: Quat) -> Mat4 {
        TransformComponent {
            position,
            rotation,
            scale: Vec3::ONE,
        }
        .to_mat4()
    }

    #[test]
    fn gjk_intersects_overlapping_cubes() {
        let cube = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(0.5, 0.0, 0.0));

        let result = gjk_intersect(&cube, a_transform, &cube, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(!hit.simplex.is_empty());
            }
            _ => panic!("Expected intersection."),
        }
    }

    #[test]
    fn gjk_no_intersection_separated_cubes() {
        let cube = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(5.0, 0.0, 0.0));

        let result = gjk_intersect(&cube, a_transform, &cube, b_transform);
        assert_eq!(result, GjkResult::NoIntersection);
    }

    #[test]
    fn gjk_intersects_overlapping_spheres() {
        let sphere = ConvexCollider::sphere(1.5, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(2.0, 0.0, 0.0));

        let result = gjk_intersect(&sphere, a_transform, &sphere, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(!hit.simplex.is_empty());
            }
            _ => panic!("Expected intersection."),
        }
    }

    #[test]
    fn gjk_no_intersection_separated_spheres() {
        let sphere = ConvexCollider::sphere(1.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(3.5, 0.0, 0.0));

        let result = gjk_intersect(&sphere, a_transform, &sphere, b_transform);
        assert_eq!(result, GjkResult::NoIntersection);
    }

    #[test]
    fn gjk_intersects_overlapping_cuboids() {
        let aabb = AABB {
            min: Vec3::splat(-2.0),
            max: Vec3::splat(1.0),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(1.0, 0.0, 0.0));

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(!hit.simplex.is_empty());
            }
            _ => panic!("Expected intersection."),
        }
    }

    #[test]
    fn gjk_no_intersection_separated_cuboids() {
        let aabb = AABB {
            min: Vec3::splat(-2.0),
            max: Vec3::splat(1.0),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(3.5, 0.0, 0.0));

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        assert_eq!(result, GjkResult::NoIntersection);
    }

    #[test]
    fn gjk_intersects_cuboid_sphere() {
        let aabb = AABB {
            min: Vec3::splat(-1.0),
            max: Vec3::splat(1.0),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let sphere = ConvexCollider::sphere(1.5, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(1.0, 0.0, 0.0));

        let result = gjk_intersect(&cuboid, a_transform, &sphere, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(!hit.simplex.is_empty());
            }
            _ => panic!("Expected intersection."),
        }
    }

    #[test]
    fn gjk_no_intersection_cuboid_sphere() {
        let aabb = AABB {
            min: Vec3::splat(-1.0),
            max: Vec3::splat(1.0),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let sphere = ConvexCollider::sphere(1.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(3.1, 0.0, 0.0));

        let result = gjk_intersect(&cuboid, a_transform, &sphere, b_transform);
        assert_eq!(result, GjkResult::NoIntersection);
    }

    #[test]
    fn gjk_intersects_rotated_cuboids() {
        let aabb = AABB {
            min: Vec3::new(-3.0, -0.25, -0.25),
            max: Vec3::new(3.0, 0.25, 0.25),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let a_transform = transform_at_with_rotation(Vec3::ZERO, Quat::from_rotation_z(0.0));
        let b_transform = transform_at_with_rotation(
            Vec3::new(0.0, 1.0, 0.0),
            Quat::from_rotation_z(90.0_f32.to_radians()),
        );

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(!hit.simplex.is_empty());
            }
            _ => panic!("Expected intersection."),
        }
    }

    #[test]
    fn gjk_no_intersection_rotated_cuboids() {
        let aabb = AABB {
            min: Vec3::new(-2.0, -1.0, -0.5),
            max: Vec3::new(2.0, 1.0, 0.5),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let a_transform = transform_at_with_rotation(Vec3::ZERO, Quat::from_rotation_z(0.0));
        let b_transform = transform_at_with_rotation(
            Vec3::new(6.0, 0.0, 0.0),
            Quat::from_rotation_z(45.0_f32.to_radians()),
        );

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        assert_eq!(result, GjkResult::NoIntersection);
    }

    #[test]
    fn gjk_intersects_x_rotated_cuboids() {
        let aabb = AABB {
            min: Vec3::new(-0.25, -3.0, -0.25),
            max: Vec3::new(0.25, 3.0, 0.25),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let a_transform = transform_at_with_rotation(Vec3::ZERO, Quat::from_rotation_x(0.0));
        let b_transform = transform_at_with_rotation(
            Vec3::new(0.0, 0.0, 1.0),
            Quat::from_rotation_x(90.0_f32.to_radians()),
        );

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(!hit.simplex.is_empty());
            }
            _ => panic!("Expected intersection."),
        }
    }

    #[test]
    fn gjk_no_intersection_x_rotated_cuboids() {
        let aabb = AABB {
            min: Vec3::new(-2.0, -1.0, -0.5),
            max: Vec3::new(2.0, 1.0, 0.5),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let a_transform = transform_at_with_rotation(Vec3::ZERO, Quat::from_rotation_x(0.0));
        let b_transform = transform_at_with_rotation(
            Vec3::new(0.0, 0.0, 3.0),
            Quat::from_rotation_x(30.0_f32.to_radians()),
        );

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        assert_eq!(result, GjkResult::NoIntersection);
    }

    #[test]
    fn gjk_intersects_y_rotated_cuboids() {
        let aabb = AABB {
            min: Vec3::new(-0.25, -0.25, -3.0),
            max: Vec3::new(0.25, 0.25, 3.0),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let a_transform = transform_at_with_rotation(Vec3::ZERO, Quat::from_rotation_y(0.0));
        let b_transform = transform_at_with_rotation(
            Vec3::new(1.0, 0.0, 0.0),
            Quat::from_rotation_y(90.0_f32.to_radians()),
        );

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(!hit.simplex.is_empty());
            }
            _ => panic!("Expected intersection."),
        }
    }

    #[test]
    fn gjk_no_intersection_y_rotated_cuboids() {
        let aabb = AABB {
            min: Vec3::new(-2.0, -1.0, -0.5),
            max: Vec3::new(2.0, 1.0, 0.5),
        };
        let cuboid = ConvexCollider::cuboid_from_aabb(aabb, CollisionLayer::Default);
        let a_transform = transform_at_with_rotation(Vec3::ZERO, Quat::from_rotation_y(0.0));
        let b_transform = transform_at_with_rotation(
            Vec3::new(6.0, 0.0, 0.0),
            Quat::from_rotation_y(30.0_f32.to_radians()),
        );

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        assert_eq!(result, GjkResult::NoIntersection);
    }

    #[test]
    fn gjk_intersects_non_uniform_cuboids() {
        // Long thin cuboid (6×1×1) vs same, separated by 4 along X → overlap = 2
        let cuboid = ConvexCollider::cuboid(Vec3::new(6.0, 1.0, 1.0), CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(4.0, 0.0, 0.0));

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(hit.simplex.len() == 4);
            }
            _ => panic!("Expected intersection for non-uniform cuboids."),
        }
    }

    #[test]
    fn gjk_no_intersection_non_uniform_cuboids() {
        let cuboid = ConvexCollider::cuboid(Vec3::new(6.0, 1.0, 1.0), CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(7.0, 0.0, 0.0));

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        assert_eq!(result, GjkResult::NoIntersection);
    }

    #[test]
    fn gjk_intersects_non_uniform_rotated() {
        // Tall thin cuboid (1×1×6) rotated 90° around Y, should intersect a cuboid at x=2
        let cuboid = ConvexCollider::cuboid(Vec3::new(1.0, 1.0, 6.0), CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at_with_rotation(
            Vec3::new(2.0, 0.0, 0.0),
            Quat::from_rotation_y(90.0_f32.to_radians()),
        );

        let result = gjk_intersect(&cuboid, a_transform, &cuboid, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(hit.simplex.len() == 4);
            }
            _ => panic!("Expected intersection for rotated non-uniform cuboids."),
        }
    }

    #[test]
    fn gjk_simplex_has_at_least_two_points() {
        // Ensure GJK always returns a simplex with >= 2 points for EPA
        let cube = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(0.5, 0.0, 0.0));

        let result = gjk_intersect(&cube, a_transform, &cube, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(
                    hit.simplex.len() == 4,
                    "GJK should always return a tetrahedron for EPA, got {}",
                    hit.simplex.len()
                );
            }
            _ => panic!("Expected intersection."),
        }
    }

    #[test]
    fn gjk_coincident_centers() {
        // Both objects at the same position (deep penetration edge case)
        let cube = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::ZERO);

        let result = gjk_intersect(&cube, a_transform, &cube, b_transform);
        match result {
            GjkResult::Intersection(hit) => {
                assert!(
                    hit.simplex.len() == 4,
                    "GJK should always return a tetrahedron even for coincident centers, got {}",
                    hit.simplex.len()
                );
            }
            _ => panic!("Expected intersection for coincident cubes."),
        }
    }

    #[test]
    fn gjk_coplaner_faces_cuboids() {
        let cube = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(0.0, 0.0, 2.0)); // Just touching along Z

        let result = gjk_intersect(&cube, a_transform, &cube, b_transform);
        dbg!(&result);
        match result {
            GjkResult::Intersection(hit) => {
                println!("GJK simplex size: {:?}", hit.simplex.len());
                assert!(
                    hit.simplex.len() == 4,
                    "GJK should always return a tetrahedron even for nearly coplanar faces, got {}",
                    hit.simplex.len()
                );
            }
            _ => panic!("Expected intersection for nearly coplanar faces."),
        }
    }

    #[test]
    fn gjk_nearly_coplaner_faces_cuboids() {
        let cube = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let m_epsilon = EPSILON * 100.0;
        let b_transform = transform_at(Vec3::new(0.0, 0.0, 2.0 - m_epsilon)); // Just touching along Z

        let result = gjk_intersect(&cube, a_transform, &cube, b_transform);
        dbg!(&result);
        match result {
            GjkResult::Intersection(hit) => {
                println!("GJK simplex size: {:?}", hit.simplex.len());
                assert!(
                    hit.simplex.len() == 4,
                    "GJK should always return a tetrahedron even for nearly coplanar faces, got {}",
                    hit.simplex.len()
                );
            }
            _ => panic!("Expected intersection for nearly coplanar faces."),
        }
    }

    #[test]
    fn gjk_should_make_tetrahedron() {
        let size = 2.0;
        let diff = 0.1;
        let cube = ConvexCollider::cube(size, CollisionLayer::Default);
        let a_transform = transform_at(Vec3::ZERO);
        let b_transform = transform_at(Vec3::new(0.0, 0.0, size - diff));

        let result = gjk_intersect(&cube, a_transform, &cube, b_transform);
        dbg!(&result);
        match result {
            GjkResult::Intersection(hit) => {
                println!("GJK simplex size: {:?}", hit.simplex.len());
                assert!(
                    hit.simplex.len() == 4,
                    "GJK simplex should have 4 points for a tetrahedron, got {}",
                    hit.simplex.len()
                );
            }
            _ => panic!("Expected intersection for 0.1 penetration."),
        }
    }
}
