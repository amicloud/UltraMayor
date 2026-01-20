use crate::{body::Body, mesh::SimpleVertex};
use nalgebra::{UnitQuaternion, Vector3};
use std::collections::{HashMap, HashSet};

pub struct MeshIslandAnalyzer;
impl MeshIslandAnalyzer {
    pub fn analyze_islands(body: &Body) -> (Vec<SimpleVertex>, Vec<u32>) {
        let mesh = &body.mesh;
        let up_direction = Vector3::new(0.0, 0.0, -1.0); // Negative Z is up
        let build_platform_z = 0.0; // Assuming build platform is at z = 0

        // Step 1: Build a mapping from each vertex to its connected vertices
        let mut vertex_connections: HashMap<u32, HashSet<u32>> = HashMap::new();

        // Populate the connections based on mesh indices (triangles)
        for i in (0..mesh.simple_indices.len()).step_by(3) {
            let v0 = mesh.simple_indices[i];
            let v1 = mesh.simple_indices[i + 1];
            let v2 = mesh.simple_indices[i + 2];

            // For each vertex in the triangle, add the other two as connected vertices
            vertex_connections
                .entry(v0)
                .or_default()
                .extend([v1, v2].iter().cloned());
            vertex_connections
                .entry(v1)
                .or_default()
                .extend([v0, v2].iter().cloned());
            vertex_connections
                .entry(v2)
                .or_default()
                .extend([v0, v1].iter().cloned());
        }

        // Ensure all vertices are present in the connections map
        for vertex_index in 0..mesh.simple_vertices.len() as u32 {
            vertex_connections.entry(vertex_index).or_default();
        }

        // Step 2: Identify potential island vertices
        let mut islands: HashSet<SimpleVertex> = HashSet::new();
        let mut island_simple_indices = HashSet::new();

        for (&vertex_index, connected_vertices) in &vertex_connections {
            // let v0: &SimpleVertex = &mesh.simple_vertices[vertex_index as usize];
            let vertex = &mesh.simple_vertices[vertex_index as usize].apply_rotation(UnitQuaternion::from_quaternion(body.rotation));

            // Exclude vertices on the build platform
            let z_delta = (vertex.position[2] - build_platform_z + body.position.z).abs();
            if z_delta.abs() < 0.001 {
                continue;
            }

            let mut is_island = true;

            for &connected_index in connected_vertices {
                let connected_vertex = &mesh.simple_vertices[connected_index as usize].apply_rotation(UnitQuaternion::from_quaternion(body.rotation));

                // Compute the direction vector from current vertex to connected vertex
                let direction = Vector3::new(
                    vertex.position[0] - connected_vertex.position[0],
                    vertex.position[1] - connected_vertex.position[1],
                    vertex.position[2] - connected_vertex.position[2],
                );

                // Avoid zero-length vectors
                if direction.norm() == 0.0 {
                    continue; // Ignore zero-length edges
                }

                // Normalize the direction
                let normalized_direction = direction.normalize();

                // Compute the dot product with the up_direction
                let dot = normalized_direction.dot(&up_direction);

                // If any edge points downwards (dot < 0), it's not an island
                if dot < 0.0 {
                    is_island = false;
                    break;
                }

                // If the edge is horizontal, we should check its connected vertices as well
                // here is where I want you to insert the code, only in this if statement!
                if dot == 0.0 {
                    // Check if the connected vertex has any edges pointing downwards
                    let connected_vertex_connections = &vertex_connections[&connected_index];
                    let mut has_downward_edge = false;

                    for &cc_index in connected_vertex_connections {
                        if cc_index == vertex_index {
                            continue; // Skip back to the original vertex
                        }

                        let cc_vertex = &mesh.simple_vertices[cc_index as usize].apply_rotation(UnitQuaternion::from_quaternion(body.rotation));

                        // Compute the direction vector from connected vertex to cc_vertex
                        let cc_direction = Vector3::new(
                            connected_vertex.position[0] - cc_vertex.position[0],
                            connected_vertex.position[1] - cc_vertex.position[1],
                            connected_vertex.position[2] - cc_vertex.position[2],
                        );

                        // Avoid zero-length vectors
                        if cc_direction.norm() == 0.0 {
                            continue; // Ignore zero-length edges
                        }

                        // Normalize the direction
                        let cc_normalized_direction = cc_direction.normalize();

                        // Compute the dot product with the up_direction
                        let cc_dot = cc_normalized_direction.dot(&up_direction);

                        // If any connected edge points downwards, the current vertex is supported
                        if cc_dot < 0.0 {
                            has_downward_edge = true;
                            break;
                        }
                    }

                    if has_downward_edge {
                        // The vertex is supported via this coplanar edge
                        is_island = false;
                        break;
                    }
                }
            }

            if is_island {
                // Insert into potential islands (HashSet ensures uniqueness based on PartialEq and Hash)
                islands.insert(*vertex);
                island_simple_indices.insert(vertex_index);
            }
        }

        // Step 3: Convert the HashSet to a Vec for the result
        (islands.into_iter().collect(),island_simple_indices.into_iter().collect())
    }
}

#[cfg(test)]
mod tests {

    use crate::{mesh::Mesh, stl_processor::StlProcessor};

    use super::*;

    #[test]
    fn test_from_stl_flat_overhang() {
        let filename = "test_stls/flat_overhang_4_points.stl";
        let processor = StlProcessor::new();
        let mut mesh = Mesh::default();
        mesh.import_stl(filename, &processor);
        let body = Body::new(mesh);
        let islands = MeshIslandAnalyzer::analyze_islands(&body);
        // let v0 = [1.601282, 18.610937, 8.000000];
        // let v1 = [1.601282, 21.813501, 8.000000];
        // let v2 = [-1.601282, 18.610937, 8.000000];
        // let v3 = [-1.601282, 21.813501, 8.000000];
        // let expected_islands = [v0, v1, v2, v3];
        islands.0
            .iter()
            .for_each(|el| println!("{:?}", el.get_position_vector3()));
        assert_eq!(
            islands.0.len(),
            4,
            "Expected 4 islands, but found: {:?}",
            islands.0.len()
        );
    }

    #[test]
    fn test_from_stl_pointed_overhang_1_point() {
        let filename = "test_stls/pointed_overhang_1_point.stl";
        let processor = StlProcessor::new();
        let mut mesh = Mesh::default();
        mesh.import_stl(filename, &processor);
        let body = Body::new(mesh);
        let islands = MeshIslandAnalyzer::analyze_islands(&body);

        islands.0.iter().for_each(|el| println!("{:?}", el.position));
        assert_eq!(
            islands.0.len(),
            1,
            "Expected 1 islands, but found: {:?}",
            islands.0.len()
        );
    }

    #[test]
    fn test_from_stl_pointed_overhang_2_points() {
        let filename = "test_stls/pointed_overhang_2_points.stl";
        let processor: StlProcessor = StlProcessor::new();
        let mut mesh = Mesh::default();
        mesh.import_stl(filename, &processor);
        let body = Body::new(mesh);
        let islands = MeshIslandAnalyzer::analyze_islands(&body);

        islands.0.iter().for_each(|el| println!("{:?}", el.position));
        assert_eq!(
            islands.0.len(),
            2,
            "Expected 2 islands, but found: {:?}",
            islands.0.len()
        );
    }

    /// This tests that the analyzer is applying the body's rotation 
    #[test]
    fn test_from_stl_pointed_overhang_1_point_rotated() {
        let filename = "test_stls/pointed_overhang_1_point.stl";
        let processor = StlProcessor::new();
        let mut mesh = Mesh::default();
        mesh.import_stl(filename, &processor);
        let mut body = Body::new(mesh);
        body.set_rotation(Vector3::new(-90.0,0.0,0.0));
        body.set_position(Vector3::new(0.0,0.0,12.5));
        let islands = MeshIslandAnalyzer::analyze_islands(&body);

        islands.0.iter().for_each(|el| println!("{:?}", el.position));
        assert_eq!(
            islands.0.len(),
            0,
            "Expected 0 islands, but found: {:?}",
            islands.0.len()
        );
    }
}
