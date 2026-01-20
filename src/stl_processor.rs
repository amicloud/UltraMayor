// src/stl_processor.rs
// Distributed under the GNU Affero General Public License v3.0 or later.
// See accompanying file LICENSE or https://www.gnu.org/licenses/agpl-3.0.html for details.

use std::io::BufReader;
use std::{ffi::OsStr, fs::File};
use stl_io::{self, Triangle};
pub struct StlProcessor;

// Define a trait for processing STL files
pub trait StlProcessorTrait {
    fn read_stl(&self, filename: &OsStr) -> Result<Vec<Triangle>, std::io::Error>;
}

// Implement the trait for the actual `StlProcessor`
impl StlProcessorTrait for StlProcessor {
    fn read_stl(&self, filename: &OsStr) -> Result<Vec<Triangle>, std::io::Error> {
        StlProcessor::read_stl(filename)
    }
}
impl StlProcessor {
    pub fn new() -> Self {
        Self {}
    }
    // Read the STL file and return the list of triangles
    pub fn read_stl(filename: &OsStr) -> Result<Vec<Triangle>, std::io::Error> {
        let file = File::open(filename)?;
        let mut reader = BufReader::new(file);
        let indexed_mesh = stl_io::read_stl(&mut reader)?;

        // Convert IndexedMesh into Vec<Triangle>
        let triangles = indexed_mesh
            .faces
            .iter()
            .map(|face| {
                let vertices = [
                    indexed_mesh.vertices[face.vertices[0]],
                    indexed_mesh.vertices[face.vertices[1]],
                    indexed_mesh.vertices[face.vertices[2]],
                ];
                Triangle {
                    normal: face.normal,
                    vertices,
                }
            })
            .collect();

        Ok(triangles)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use stl_io::Triangle;
    use tempfile::NamedTempFile;

    // Helper function to create a Triangle
    fn create_triangle(v0: [f32; 3], v1: [f32; 3], v2: [f32; 3], normal: [f32; 3]) -> Triangle {
        Triangle {
            normal,
            vertices: [v0, v1, v2],
        }
    }

    // Helper function to write ASCII STL content to a temporary file
    fn write_ascii_stl(triangles: &[Triangle]) -> NamedTempFile {
        let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");
        writeln!(temp_file, "solid test").expect("Failed to write to temp file");
        for tri in triangles {
            writeln!(
                temp_file,
                "  facet normal {} {} {}",
                tri.normal[0], tri.normal[1], tri.normal[2]
            )
            .expect("Failed to write to temp file");
            writeln!(temp_file, "    outer loop").expect("Failed to write to temp file");
            for vertex in &tri.vertices {
                writeln!(
                    temp_file,
                    "      vertex {} {} {}",
                    vertex[0], vertex[1], vertex[2]
                )
                .expect("Failed to write to temp file");
            }
            writeln!(temp_file, "    endloop").expect("Failed to write to temp file");
            writeln!(temp_file, "  endfacet").expect("Failed to write to temp file");
        }
        writeln!(temp_file, "endsolid test").expect("Failed to write to temp file");
        temp_file
    }

    // Helper function to serialize [f32; 3] to Vec<u8> in little-endian
    fn serialize_f32_array(arr: &[f32; 3]) -> Vec<u8> {
        arr.iter().flat_map(|&num| num.to_le_bytes()).collect()
    }

    // Helper function to write binary STL content to a temporary file
    fn write_binary_stl(triangles: &[Triangle]) -> NamedTempFile {
        let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");

        // Write a simple binary STL header (80 bytes)
        let header = [0u8; 80];
        temp_file
            .write_all(&header)
            .expect("Failed to write header");

        // Number of triangles (4 bytes, little endian)
        let num_triangles = triangles.len() as u32;
        temp_file
            .write_all(&num_triangles.to_le_bytes())
            .expect("Failed to write triangle count");

        // Write each triangle
        for tri in triangles {
            // Serialize and write the normal
            let normal_bytes = serialize_f32_array(&tri.normal);
            temp_file
                .write_all(&normal_bytes)
                .expect("Failed to write normal");

            // Serialize and write each vertex
            for vertex in &tri.vertices {
                let vertex_bytes = serialize_f32_array(vertex);
                temp_file
                    .write_all(&vertex_bytes)
                    .expect("Failed to write vertex");
            }

            // Attribute byte count (2 bytes), usually zero
            temp_file
                .write_all(&[0u8; 2])
                .expect("Failed to write attribute byte count");
        }

        temp_file
    }

    #[test]
    fn test_read_stl_ascii_valid() {
        // Arrange: Define known triangles
        let triangles = vec![
            create_triangle(
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ),
            create_triangle(
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ),
        ];

        // Write to a temporary ASCII STL file
        let temp_file = write_ascii_stl(&triangles);

        // Act: Read the STL file
        let processor = StlProcessor::new();
        let result = processor.read_stl(temp_file.path().as_os_str());

        // Assert: Ensure no errors and triangles match
        assert!(
            result.is_ok(),
            "read_stl should return Ok for valid ASCII STL"
        );
        let read_triangles = result.unwrap();
        assert_eq!(
            read_triangles.len(),
            triangles.len(),
            "Number of triangles should match"
        );

        for (read_tri, expected_tri) in read_triangles.iter().zip(triangles.iter()) {
            assert_eq!(
                read_tri.normal, expected_tri.normal,
                "Triangle normals should match"
            );
            assert_eq!(
                read_tri.vertices, expected_tri.vertices,
                "Triangle vertices should match"
            );
        }
    }

    #[test]
    fn test_read_stl_binary_valid() {
        // Arrange: Define known triangles
        let triangles = vec![
            create_triangle(
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ),
            create_triangle(
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ),
        ];

        // Write to a temporary binary STL file
        let temp_file = write_binary_stl(&triangles);

        // Act: Read the STL file
        let processor = StlProcessor::new();
        let result = processor.read_stl(temp_file.path().as_os_str());

        // Assert: Ensure no errors and triangles match
        assert!(
            result.is_ok(),
            "read_stl should return Ok for valid binary STL"
        );
        let read_triangles = result.unwrap();
        assert_eq!(
            read_triangles.len(),
            triangles.len(),
            "Number of triangles should match"
        );

        for (read_tri, expected_tri) in read_triangles.iter().zip(triangles.iter()) {
            assert_eq!(
                read_tri.normal, expected_tri.normal,
                "Triangle normals should match"
            );
            assert_eq!(
                read_tri.vertices, expected_tri.vertices,
                "Triangle vertices should match"
            );
        }
    }

    #[test]
    fn test_read_stl_nonexistent_file() {
        // Arrange: Define a filename that doesn't exist
        let filename = OsStr::new("/do/not/ever/make/a/file/here/or/you/will/be/cursed");

        // Act: Attempt to read the non-existent file
        let processor = StlProcessor::new();
        let result = processor.read_stl(filename);

        // Assert: Ensure an error is returned
        assert!(
            result.is_err(),
            "read_stl should return Err for non-existent files"
        );
    }

    #[test]
    fn test_read_stl_ascii_malformed() {
        // Arrange: Create a temporary malformed ASCII STL file
        let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");
        let malformed_content = "\
            solid test
              facet normal 0 0 1
                outer loop
                  vertex 0 0 0
                  vertex 1 0 0
                  // Missing the third vertex
                endloop
              endfacet
            endsolid test
        ";
        write!(temp_file, "{}", malformed_content).expect("Failed to write to temp file");

        // Act: Attempt to read the malformed STL file
        let processor = StlProcessor::new();
        let result = processor.read_stl(temp_file.path().as_os_str());

        // Assert: Ensure an error is returned
        assert!(
            result.is_err(),
            "read_stl should return Err for malformed ASCII STL files"
        );
    }

    #[test]
    fn test_read_stl_empty_file() {
        // Arrange: Create a temporary empty STL file
        let temp_file = NamedTempFile::new().expect("Failed to create temp file");

        // Act: Attempt to read the empty STL file
        let processor = StlProcessor::new();
        let result = processor.read_stl(temp_file.path().as_os_str());

        // Assert: Ensure an error is returned (since the STL content is invalid)
        assert!(
            result.is_err(),
            "read_stl should return Err for empty STL files"
        );
    }

    #[test]
    fn test_read_stl_ascii_no_triangles() {
        // Arrange: Create a temporary ASCII STL file with no triangles
        let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");
        let stl_content = "\
            solid empty
            endsolid empty
        ";
        write!(temp_file, "{}", stl_content).expect("Failed to write to temp file");

        // Act: Read the STL file
        let processor = StlProcessor::new();
        let result = processor.read_stl(temp_file.path().as_os_str());

        // Assert: Ensure no errors and no triangles are parsed
        assert!(
            result.is_ok(),
            "read_stl should return Ok for STL files with no triangles"
        );
        let read_triangles = result.unwrap();
        assert!(
            read_triangles.is_empty(),
            "read_stl should return an empty vector for STL files with no triangles"
        );
    }

    #[test]
    fn test_read_stl_binary_no_triangles() {
        // Arrange: Create a temporary binary STL file with zero triangles
        let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");
        // Write a simple binary STL header (80 bytes)
        let header = [0u8; 80];
        temp_file
            .write_all(&header)
            .expect("Failed to write header");
        // Number of triangles (4 bytes, little endian) set to zero
        let num_triangles = 0u32;
        temp_file
            .write_all(&num_triangles.to_le_bytes())
            .expect("Failed to write triangle count");

        // Act: Read the STL file
        let processor = StlProcessor::new();
        let result = processor.read_stl(temp_file.path().as_os_str());

        // Assert: Ensure no errors and no triangles are parsed
        assert!(
            result.is_ok(),
            "read_stl should return Ok for binary STL files with zero triangles"
        );
        let read_triangles = result.unwrap();
        assert!(
            read_triangles.is_empty(),
            "read_stl should return an empty vector for binary STL files with zero triangles"
        );
    }

    #[test]
    fn test_read_stl_binary_malformed() {
        // Arrange: Create a temporary malformed binary STL file
        let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");
        // Write a malformed binary STL header (less than 80 bytes)
        let header = [0u8; 10];
        temp_file
            .write_all(&header)
            .expect("Failed to write header");
        // Write an incorrect number of triangles (e.g., 1 but no triangle data)
        let num_triangles = 1u32;
        temp_file
            .write_all(&num_triangles.to_le_bytes())
            .expect("Failed to write triangle count");

        // Act: Attempt to read the malformed binary STL file
        let processor = StlProcessor::new();
        let result = processor.read_stl(temp_file.path().as_os_str());

        // Assert: Ensure an error is returned due to insufficient data
        assert!(
            result.is_err(),
            "read_stl should return Err for malformed binary STL files"
        );
    }
}
