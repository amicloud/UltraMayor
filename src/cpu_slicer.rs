// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use crate::body::Body;
use crate::printer::Printer;
use geo::algorithm::area::Area;
use geo::{Contains, Coord, Line, LineString, Polygon};
use image::{ImageBuffer, ImageError, Luma};
use imageproc::drawing::draw_polygon_mut;
use imageproc::point::Point;
use log::debug;
use nalgebra::{OPoint, Vector3};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use std::collections::{HashMap, HashSet};
use stl_io::{self, Triangle};
use thiserror::Error;
// use geo_types::line_string;
use geo::algorithm::intersects::Intersects; // Provides intersects method for line strings

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Orientation {
    INSIDE,
    OUTSIDE,
}

#[derive(Default)]
pub struct CPUSlicer {}

impl CPUSlicer {
    pub fn slice_bodies(
        bodies: Vec<Body>,
        slice_thickness: f64,
        printer: &Printer,
    ) -> Result<Vec<ImageBuffer<Luma<u8>, Vec<u8>>>, CPUSlicerError> {
        let mut triangles: Vec<Triangle> = Vec::new();

        for mut body in bodies {
            body.mesh.get_triangles_for_slicing();
            let model_matrix = body.get_model_matrix();

            for tri in &body.mesh.get_triangles_for_slicing() {
                // Convert each vertex from [f32; 3] to OPoint<f32, 3>
                let vertex0 = OPoint::from(tri.vertices[0]);
                let vertex1 = OPoint::from(tri.vertices[1]);
                let vertex2 = OPoint::from(tri.vertices[2]);

                // Transform each vertex using the model matrix
                let transformed_vertex0 = model_matrix.transform_point(&vertex0).coords.into();
                let transformed_vertex1 = model_matrix.transform_point(&vertex1).coords.into();
                let transformed_vertex2 = model_matrix.transform_point(&vertex2).coords.into();

                let transformed_vertices = [
                    transformed_vertex0,
                    transformed_vertex1,
                    transformed_vertex2,
                ];

                // Convert normal from [f32; 3] to Vector3<f32>
                let normal_vector = Vector3::from(tri.normal);
                // println!("Position vector for triangle: {:?}", transformed_vertices4);
                // Transform and normalize the normal vector
                let transformed_normal = model_matrix.transform_vector(&normal_vector).normalize();
                // println!("Normal Vector for triangle: {:?}", normal_vector);

                // Convert the transformed normal back to [f32; 3]
                let transformed_normal_array: [f32; 3] = transformed_normal.into();

                // Create a new Triangle with transformed data
                let transformed_triangle = Triangle {
                    normal: transformed_normal_array,
                    vertices: transformed_vertices,
                };

                // Add the transformed triangle to the list
                triangles.push(transformed_triangle);
            }
        }
        Self::generate_slice_images(&triangles, slice_thickness, printer)
    }

    fn generate_slice_images(
        triangles: &[Triangle],
        slice_thickness: f64,
        printer: &Printer,
    ) -> Result<Vec<ImageBuffer<Luma<u8>, Vec<u8>>>, CPUSlicerError> {
        let (min_z, max_z) = CPUSlicer::z_range(triangles);
        let mut slice_z_values = Vec::new();
        let mut z = min_z;
        while z <= max_z {
            slice_z_values.push(z);
            z += slice_thickness;
        }

        let images: Vec<ImageBuffer<Luma<u8>, Vec<u8>>> = slice_z_values
            .par_iter()
            .filter_map(|plane_z| {
                let segments = CPUSlicer::collect_intersection_segments(triangles, *plane_z);
                if segments.is_empty() {
                    return None;
                }

                let raw_polygons = CPUSlicer::assemble_polygons(&segments);
                if raw_polygons.is_empty() {
                    return None;
                }

                let mut image =
                    ImageBuffer::from_pixel(printer.pixel_x, printer.pixel_y, Luma([0u8]));

                // Now using classify_and_structure_polygons with depth information
                let (exterior_with_depth, holes_with_depth) =
                    Self::classify_and_structure_polygons(raw_polygons);

                // Combine exteriors and holes into one list for rendering
                let mut all_polygons_with_depth: Vec<((Polygon, Orientation), usize)> =
                    exterior_with_depth;
                all_polygons_with_depth.extend(holes_with_depth);
                all_polygons_with_depth.sort_by(|a, b| {
                    // Compare depths first
                    a.1.cmp(&b.1)
                        // If depths are equal, compare orientations
                        .then_with(|| a.0 .1.cmp(&b.0 .1))
                });

                for (polygon, depth) in all_polygons_with_depth {
                    let points: Vec<Point<i32>> = polygon
                        .0
                        .exterior()
                        .points()
                        .map(|p| {
                            let (x, y) = Self::model_to_image_coords(
                                p.x(),
                                p.y(),
                                printer.pixel_x,
                                printer.physical_x,
                                printer.pixel_y,
                                printer.physical_y,
                            );
                            Point::new(x, y)
                        })
                        .collect();

                    let mut unique_points: Vec<Point<i32>> = Vec::new();

                    // Manually check for duplicates
                    for point in points {
                        // Check if the point is already in the unique_points vector
                        if !unique_points.iter().any(|p| p == &point) {
                            unique_points.push(point);
                        }
                    }

                    if unique_points.len() >= 3 {
                        match polygon.1 {
                            Orientation::INSIDE => {
                                if depth == 0 {
                                    // This really shouldn't happen but it seems there is an issue with my orientation algorithm and
                                    // this is a bandaid fix that semms to work in most cases
                                    draw_polygon_mut(&mut image, &unique_points, Luma([200u8]));
                                } else {
                                    // Draw interior polygons, holes, black (or grey for debugging)
                                    draw_polygon_mut(&mut image, &unique_points, Luma([69u8]));
                                }
                            }
                            Orientation::OUTSIDE => {
                                // Draw exterior polygons white
                                draw_polygon_mut(&mut image, &unique_points, Luma([255u8]));
                            }
                        }
                    }
                }

                Some(image)
            })
            .collect();

        Ok(images)
    }

    fn classify_and_structure_polygons(
        polygons: Vec<(Vec<Vector3<f64>>, Orientation)>,
    ) -> (
        Vec<((Polygon<f64>, Orientation), usize)>,
        Vec<((Polygon<f64>, Orientation), usize)>,
    ) {
        let mut exteriors_with_depth: Vec<((Polygon<f64>, Orientation), usize)> = Vec::new();
        let mut holes_with_depth: Vec<((Polygon<f64>, Orientation), usize)> = Vec::new();

        let mut all_polygons: Vec<(Polygon<f64>, Orientation)> = Vec::new();

        for (_i, (points, orientation)) in polygons.iter().enumerate() {
            // Convert the points to a geo::Polygon and store with its orientation
            let linestring_geo: LineString<f64> = LineString::from(
                points
                    .iter()
                    .map(|p| (p.x, p.y))
                    .collect::<Vec<(f64, f64)>>(),
            );
            let polygon = Polygon::new(linestring_geo, vec![]);
            all_polygons.push((polygon, *orientation));
        }

        // Now check nesting and assign depths
        for (i, (polygon, orientation)) in all_polygons.iter().enumerate() {
            let mut depth = 0;
            for (j, (other_polygon, _)) in all_polygons.iter().enumerate() {
                if i != j && Self::is_polygon_inside(polygon, other_polygon) {
                    depth += 1;
                }
            }

            println!("Signed area: {:?}", polygon.signed_area());

            // Assign the polygon to the appropriate list (exterior or interior) with depth
            match orientation {
                Orientation::INSIDE => {
                    holes_with_depth.push(((polygon.clone(), *orientation), depth));
                    println!("Pushing inside polygon with depth: {}", depth);
                    if depth == 0 {
                        println!("Inside polygon with depth 0. BAD!");
                    }
                }
                Orientation::OUTSIDE => {
                    exteriors_with_depth.push(((polygon.clone(), *orientation), depth));
                    println!("Pushing outside polygon with depth: {}", depth);
                    // It is okay to have outside polygons of any depth
                }
            }
        }

        (exteriors_with_depth, holes_with_depth)
    }

    fn is_polygon_inside(polygon: &Polygon<f64>, other_polygon: &Polygon<f64>) -> bool {
        // First, check if all vertices of `polygon` are inside `other_polygon`
        for point in polygon.exterior().points() {
            if !other_polygon.contains(&point) {
                return false; // If any vertex is outside, return false
            }
        }

        // Prepare to check if the edges of `polygon` intersect with the edges of `other_polygon`
        let polygon_lines: Vec<Line<f64>> = polygon.exterior().lines().collect(); // Collecting lines for the edges of the polygon
        let other_polygon_lines: Vec<Line<f64>> = other_polygon.exterior().lines().collect(); // Collecting lines for the edges of the other polygon

        // Check for intersections between edges
        for polygon_line in &polygon_lines {
            // Iterate over references
            for other_polygon_line in &other_polygon_lines {
                // Iterate over references
                if polygon_line.intersects(other_polygon_line) {
                    return false; // If any edges intersect, return false
                }
            }
        }

        true // All vertices are inside, and no edges intersect
    }

    /// Checks if a point should be considered part of an inside or outside segment of the polygon
    fn check_point_orientation(position: Vector3<f64>, normal: [f32; 3]) -> i32 {
        let px = position.x;
        let py = position.y;
        let nx = normal[0] as f64;
        let ny = normal[1] as f64;

        // Check if the signs match for both x and y components
        // 0 matches both signs
        let x_sign_matches = (px >= 0.0 && nx >= 0.0) || (px <= 0.0 && nx <= 0.0);
        let y_sign_matches = (py >= 0.0 && ny >= 0.0) || (py <= 0.0 && ny <= 0.0);

        // If both x and y signs match, the point is part of an outward-facing segment, otherwise inward
        if x_sign_matches && y_sign_matches {
            1 // OUTSIDE
        } else {
            -1 // INSIDE
        }
    }

    // Assembles segments into closed polygons
    fn assemble_polygons(
        segments: &[((Vector3<f64>, Vector3<f64>), [f32; 3])],
    ) -> Vec<(Vec<Vector3<f64>>, Orientation)> {
        fn point_to_key(p: &Vector3<f64>, epsilon: f64) -> (i64, i64) {
            let scale = 1.0 / epsilon;
            let x = (p[0] * scale).round() as i64;
            let y = (p[1] * scale).round() as i64;
            (x, y)
        }

        let epsilon = 1e-6;
        let mut point_coords: HashMap<(i64, i64), (Vector3<f64>, [f32; 3])> = HashMap::new();
        let mut adjacency: HashMap<(i64, i64), Vec<(i64, i64)>> = HashMap::new();

        // Build adjacency map
        for &((ref start, ref end), normal) in segments {
            let start_key = point_to_key(start, epsilon);
            let end_key = point_to_key(end, epsilon);

            point_coords
                .entry(start_key)
                .or_insert_with(|| (*start, normal));
            point_coords
                .entry(end_key)
                .or_insert_with(|| (*end, normal));

            adjacency.entry(start_key).or_default().push(end_key);
            adjacency.entry(end_key).or_default().push(start_key);
        }

        let mut polygons = Vec::new();
        let mut visited_edges: HashSet<((i64, i64), (i64, i64))> = HashSet::new();

        // Traverse the graph to assemble polygons
        for &start_key in adjacency.keys() {
            for &next_key in &adjacency[&start_key] {
                let edge = (start_key, next_key);
                if visited_edges.contains(&edge) || visited_edges.contains(&(next_key, start_key)) {
                    continue;
                }

                let mut polygon_keys = vec![start_key];
                let mut current_key = next_key;
                visited_edges.insert(edge);

                loop {
                    polygon_keys.push(current_key);

                    if let Some(neighbors) = adjacency.get(&current_key) {
                        // Find the next neighbor that hasn't been visited
                        let mut found = false;
                        for &neighbor_key in neighbors {
                            let edge = (current_key, neighbor_key);
                            if neighbor_key != polygon_keys[polygon_keys.len() - 2]
                                && !visited_edges.contains(&edge)
                                && !visited_edges.contains(&(neighbor_key, current_key))
                            {
                                visited_edges.insert(edge);
                                current_key = neighbor_key;
                                found = true;
                                break;
                            }
                        }

                        if !found {
                            break;
                        }

                        // Check if the polygon is closed
                        if current_key == start_key {
                            break;
                        }
                    } else {
                        break;
                    }
                }

                // If we have a closed polygon
                if polygon_keys.len() >= 3 && current_key == start_key {
                    // Convert keys back to points
                    let polygon: Vec<Vector3<f64>> =
                        polygon_keys.iter().map(|key| point_coords[key].0).collect();
                    let normals: Vec<[f32; 3]> = polygon
                        .iter()
                        .map(|point| point_coords[&point_to_key(point, epsilon)].1)
                        .collect();

                    // **Calculate the centroid of the polygon**
                    let centroid = Self::calculate_centroid(&polygon);

                    // **Sum the orientations of the points offset by the centroid, weighted by segment length**
                    let mut orientation_sum = 0.0;

                    for i in 0..polygon.len() {
                        let point = polygon[i];
                        let last_point = if i == 0 {
                            polygon[polygon.len() - 1] // Wrap around to the last point for the first point
                        } else {
                            polygon[i - 1] // Normal case: get the previous point
                        };

                        // Calculate the segment length between the current point and the last point
                        let segment_length = (point - last_point).norm();

                        // Offset the current point by subtracting the centroid
                        let offset_point = point - centroid;

                        // Calculate the weighted orientation, multiplying by the segment length
                        orientation_sum += Self::check_point_orientation(offset_point, normals[i])
                            as f64
                            * segment_length;
                    }

                    // Decide if the polygon is an exterior or a hole based on the sum of orientations
                    let orientation = if orientation_sum >= 0.0 {
                        Orientation::OUTSIDE
                    } else {
                        Orientation::INSIDE
                    };
                    polygons.push((polygon, orientation));
                }
            }
        }
        polygons
    }

    /// Calculates the centroid of a polygon (assumes a 2D polygon in 3D space)
    fn calculate_centroid(polygon: &[Vector3<f64>]) -> Vector3<f64> {
        let mut centroid = Vector3::new(0.0, 0.0, 0.0);
        for point in polygon {
            centroid += *point;
        }
        centroid /= polygon.len() as f64;
        centroid
    }

    // Determine the Z-axis range of the model
    fn z_range(triangles: &[Triangle]) -> (f64, f64) {
        let z_coords: Vec<f64> = triangles
            .iter()
            .flat_map(|tri| tri.vertices.iter().map(|v| v[2] as f64))
            .collect();

        let min_z = z_coords.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_z = z_coords.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        (min_z, max_z)
    }

    // Compute the intersection of a triangle with a horizontal plane at z = plane_z
    fn intersect_triangle_with_plane(triangle: &Triangle, plane_z: f64) -> Vec<Vector3<f64>> {
        let epsilon = 1e-6; // Tolerance for floating-point comparisons

        let points: Vec<Vector3<f64>> = triangle
            .vertices
            .iter()
            .map(|v| Vector3::new(v[0] as f64, v[1] as f64, v[2] as f64))
            .collect();

        let distances: Vec<f64> = points.iter().map(|p| p[2] - plane_z).collect();

        // Check if all points are on one side of the plane
        let positive = distances.iter().any(|&distance| distance > epsilon);
        let negative = distances.iter().any(|&distance| distance < -epsilon);

        // No intersection if all points are on one side
        if !(positive && negative) {
            // Additionally, check if the triangle is coplanar
            if distances.iter().all(|&d| d.abs() <= epsilon) {
                // Triangle is coplanar; skip it to avoid duplicate segments
                return vec![];
            }
            return vec![];
        }

        // Find intersection points
        let mut intersections = Vec::new();

        for i in 0..3 {
            let p1 = points[i];
            let p2 = points[(i + 1) % 3];
            let d1 = distances[i];
            let d2 = distances[(i + 1) % 3];

            if (d1 > epsilon && d2 < -epsilon) || (d1 < -epsilon && d2 > epsilon) {
                let t = d1 / (d1 - d2);
                let intersection = p1 + (p2 - p1) * t;
                intersections.push(intersection);
            }
        }

        // Remove duplicate points
        intersections.sort_by(|a, b| {
            a[0].partial_cmp(&b[0])
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a[1].partial_cmp(&b[1]).unwrap_or(std::cmp::Ordering::Equal))
                .then(a[2].partial_cmp(&b[2]).unwrap_or(std::cmp::Ordering::Equal))
        });
        intersections.dedup_by(|a, b| a.metric_distance(b) < epsilon);

        intersections
    }

    // Collect all intersection segments at a given plane_z
    fn collect_intersection_segments(
        triangles: &[Triangle],
        plane_z: f64,
    ) -> Vec<((Vector3<f64>, Vector3<f64>), [f32; 3])> {
        let mut segments = Vec::new();
        let mut seen_segments = HashSet::new(); // To track unique segments

        for triangle in triangles {
            let intersection_points = CPUSlicer::intersect_triangle_with_plane(triangle, plane_z);

            if intersection_points.len() == 2 {
                let mut segment = (intersection_points[0], intersection_points[1]);

                // Sort the segment endpoints to ensure consistency
                if (segment.0[0], segment.0[1]) > (segment.1[0], segment.1[1]) {
                    segment = (segment.1, segment.0);
                }

                // Create a unique key for the segment
                let key = (
                    (segment.0[0] * 1e6).round() as i64,
                    (segment.0[1] * 1e6).round() as i64,
                    (segment.1[0] * 1e6).round() as i64,
                    (segment.1[1] * 1e6).round() as i64,
                );

                if !seen_segments.contains(&key) {
                    segments.push((segment, triangle.normal)); // push the segment with the triangle's normal
                    seen_segments.insert(key);
                }
            } else if intersection_points.len() > 2 {
                debug!(
                    "Skipped a triangle intersecting the plane in multiple points at z={}",
                    plane_z
                );
            }
        }
        segments
    }

    #[allow(dead_code)]
    // Calculate the area of a polygon using the Shoelace formula
    fn polygon_area(polygon: &[Vector3<f64>]) -> f64 {
        let coords: Vec<Coord<f64>> = polygon.iter().map(|p| Coord { x: p[0], y: p[1] }).collect();

        let linestring = LineString::from(coords);
        let polygon = Polygon::new(linestring, vec![]);

        let area = polygon.unsigned_area();
        debug!("Polygon area: {} ", area);
        area
    }

    // Translates points so that that 0,0 is at the center of the image
    fn model_to_image_coords(
        x: f64,
        y: f64,
        pixel_x: u32,
        physical_x: f64,
        pixel_y: u32,
        physical_y: f64,
    ) -> (i32, i32) {
        // Calculate pixels per millimeter
        let ppm_x = pixel_x as f64 / physical_x;
        let ppm_y = pixel_y as f64 / physical_y;

        // Apply scaling
        let scaled_x = x * ppm_x;
        let scaled_y = y * ppm_y;

        // Translate coordinates to image space (centered)
        let image_x = scaled_x + (pixel_x as f64 / 2.0);
        let image_y = scaled_y + (pixel_y as f64 / 2.0);

        (image_x.round() as i32, image_y.round() as i32)
    }
}

#[derive(Error, Debug)]
pub enum CPUSlicerError {
    #[error("Image processing error: {0}")]
    ImageProcessingError(#[from] ImageError),

    #[error("Thread join error: {0}")]
    ThreadJoinError(String),
}

#[cfg(test)]
mod tests {
    use crate::mesh::Mesh;
    use crate::stl_processor::StlProcessor;

    use super::*;
    use geo::{LineString, Polygon};
    use nalgebra::Vector3;

    #[test]
    fn test_is_polygon_inside() {
        // Create an outer polygon (a square)
        let outer_polygon = Polygon::new(
            LineString::from(vec![
                (0.0, 0.0),
                (0.0, 10.0),
                (10.0, 10.0),
                (10.0, 0.0),
                (0.0, 0.0),
            ]),
            vec![],
        );

        // Create an inner polygon (a smaller square)
        let inner_polygon = Polygon::new(
            LineString::from(vec![
                (1.0, 1.0),
                (1.0, 9.0),
                (9.0, 9.0),
                (9.0, 1.0),
                (1.0, 1.0),
            ]),
            vec![],
        );

        assert!(CPUSlicer::is_polygon_inside(&inner_polygon, &outer_polygon));
        assert!(!CPUSlicer::is_polygon_inside(
            &outer_polygon,
            &inner_polygon
        ));

        // Create a non-nested polygon (overlapping but not contained)
        let overlapping_polygon = Polygon::new(
            LineString::from(vec![
                (5.0, 5.0),
                (5.0, 15.0),
                (15.0, 15.0),
                (15.0, 5.0),
                (5.0, 5.0),
            ]),
            vec![],
        );

        assert!(!CPUSlicer::is_polygon_inside(
            &overlapping_polygon,
            &outer_polygon
        ));
        assert!(!CPUSlicer::is_polygon_inside(
            &outer_polygon,
            &overlapping_polygon
        ));
    }

    #[test]
    fn test_classify_and_structure_polygons() {
        // Define a simple test for classify_and_structure_polygons
        let polygons = vec![
            (
                vec![
                    Vector3::new(0.0, 0.0, 0.0),
                    Vector3::new(0.0, 2.0, 0.0),
                    Vector3::new(2.0, 2.0, 0.0),
                    Vector3::new(2.0, 0.0, 0.0),
                ],
                Orientation::OUTSIDE,
            ),
            (
                vec![
                    Vector3::new(1.0, 1.0, 0.0),
                    Vector3::new(1.0, 3.0, 0.0),
                    Vector3::new(3.0, 3.0, 0.0),
                    Vector3::new(3.0, 1.0, 0.0),
                ],
                Orientation::INSIDE,
            ),
        ];

        let (exteriors, holes) = CPUSlicer::classify_and_structure_polygons(polygons);
        assert_eq!(exteriors.len(), 1);
        assert_eq!(holes.len(), 1);
    }

    #[test]
    fn test_slice_bodies() {
        let stl_processor = StlProcessor::new();
        let mut mesh = Mesh::default();
        mesh.import_stl("test_stls/with_holes.stl", &stl_processor);
        let body = Body::new(mesh);
        let printer = Printer::default();
        let result = CPUSlicer::slice_bodies(vec![body.clone()], 0.1, &printer);
        // it would really be nice to get some kind of data back from the slice bodies function that we can use to verify
        // the functionality in tests. It could possibly be useful for other things
        assert!(result.is_ok());

        let images = result.unwrap();
        assert!(!images.is_empty()); // Ensure that at least one image is generated
        assert_eq!(images[0].dimensions(), (printer.pixel_x, printer.pixel_y)); // Check the image dimensions
    }
}
