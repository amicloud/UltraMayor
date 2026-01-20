pub mod file_manager {
    use image::{EncodableLayout, ImageBuffer, Luma, Rgb};
    use log::debug;
    use rayon::iter::IntoParallelRefIterator;
    use rayon::prelude::*;
    use std::fs;
    use std::fs::File;
    use std::io::Write;
    use std::path::Path;
    use std::time::SystemTime;
    use std::time::UNIX_EPOCH;
    use webp::Encoder as WebpEncoder;

    use zip::result::ZipError;
    use zip::write::SimpleFileOptions;
    use zip::ZipWriter;

    use crate::cpu_slicer::CPUSlicerError;
    #[allow(dead_code)]
    pub async fn write_images_to_zip_file(
        images: &Vec<ImageBuffer<Luma<u8>, Vec<u8>>>,
    ) -> Result<String, ZipError> {
        // Get current timestamp for the zip file name
        let start = SystemTime::now();
        let since_the_epoch = start
            .duration_since(UNIX_EPOCH)
            .expect("Time went backwards");
        let timestamp = since_the_epoch.as_secs();
        // Directory path where the zip file will be saved
        let dir_path = "slices";

        // Check if the directory exists, and if not, create it
        if !Path::new(dir_path).exists() {
            fs::create_dir_all(dir_path).expect("Failed to create directory");
        }

        // Create a new zip file
        let zip_file_path: String = format!("slices/slices_{}.zip", timestamp);
        let zip_file = File::create(&zip_file_path).expect("Failed to create zip file");
        let mut zip = ZipWriter::new(zip_file);

        // Iterate over the output images and save each one to the zip file in lossless WebP format
        for (i, image) in images.iter().enumerate() {
            // Convert ImageBuffer<Luma<u8>, Vec<u8>> to ImageBuffer<Rgb<u8>, Vec<u8>>
            let rgb_image: ImageBuffer<Rgb<u8>, Vec<u8>> = convert_luma_to_rgb(image);

            // Retrieve width and height
            let width = rgb_image.width();
            let height = rgb_image.height();

            // Flatten the RGB image into a Vec<u8>
            let rgb_data = rgb_image.into_raw();

            // Create a WebP encoder with lossless encoding
            let encoder = WebpEncoder::from_rgb(&rgb_data, width, height);

            // Encode the image in lossless mode
            let webp_data = encoder.encode_lossless();
            let webp_bytes = webp_data.as_bytes();

            // Create a file entry in the zip
            let file_name = format!("slice_{:04}.webp", i);

            // Define the file options for the zip entry
            let options =
                SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored);

            // Start a new file in the zip archive
            zip.start_file(&file_name, options)?;

            // Write the image data to the zip file
            zip.write_all(webp_bytes)?; // Use write_all to ensure all data is written

            // Log progress if needed
            debug!("Added {} to zip", &file_name);
        }

        // Finish the zip file
        zip.finish()?;
        Ok(zip_file_path)
    }

    pub async fn write_webps_to_folder(
        images: &Vec<ImageBuffer<Luma<u8>, Vec<u8>>>,
    ) -> Result<String, CPUSlicerError> {
        let start = SystemTime::now();
        let since_the_epoch = start
            .duration_since(UNIX_EPOCH)
            .expect("Time went backwards");
        let timestamp = since_the_epoch.as_secs();

        // Create a new directory inside "slices" with the timestamp as its name
        let dir_path = format!("slices/{}", timestamp);
        fs::create_dir_all(&dir_path).expect("Failed to create directory");

        // Iterate over the output images and save each one to a file in lossless WebP format
        images.par_iter().enumerate().for_each(|(i, image)| {
            let file_path = format!("{}/slice_{:04}.webp", dir_path, i);

            // Convert ImageBuffer<Luma<u8>, Vec<u8>> to ImageBuffer<Rgb<u8>, Vec<u8>>
            let rgb_image: ImageBuffer<Rgb<u8>, Vec<u8>> = convert_luma_to_rgb(image);

            // Retrieve width and height before moving rgb_image
            let width = rgb_image.width();
            let height = rgb_image.height();

            // Flatten the RGB image into a Vec<u8>
            let rgb_data = rgb_image.into_raw();

            // Create a WebP encoder with lossless encoding
            let encoder = WebpEncoder::from_rgb(&rgb_data, width, height);

            // Encode the image in lossless mode
            let webp_data = encoder.encode_lossless();

            // Convert WebPMemory to Vec<u8> using as_bytes()
            let webp_bytes = webp_data.as_bytes();

            // Save the encoded WebP data to a file
            fs::write(&file_path, webp_bytes).unwrap(); // Todo: Handle this better
        });
        Ok(dir_path)
    }

    /// Converts an ImageBuffer with Luma<u8> pixels to an ImageBuffer with Rgb<u8> pixels
    pub fn convert_luma_to_rgb(
        image: &ImageBuffer<Luma<u8>, Vec<u8>>,
    ) -> ImageBuffer<Rgb<u8>, Vec<u8>> {
        let (width, height) = image.dimensions();
        let mut rgb_image = ImageBuffer::new(width, height);

        for (x, y, pixel) in image.enumerate_pixels() {
            let luma = pixel[0];
            rgb_image.put_pixel(x, y, Rgb([luma, luma, luma]));
        }

        rgb_image
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use image::{ImageBuffer, Luma};
    use std::fs;
    use std::path::Path;

    fn create_test_image(width: u32, height: u32, value: u8) -> ImageBuffer<Luma<u8>, Vec<u8>> {
        ImageBuffer::from_fn(width, height, |_, _| Luma([value]))
    }

    #[tokio::test]
    async fn test_write_images_to_zip_file() {
        let images = vec![
            create_test_image(100, 100, 255),
            create_test_image(200, 200, 128),
        ];

        let result = file_manager::write_images_to_zip_file(&images).await;

        assert!(result.is_ok());
        let zip_file_path = result.unwrap();

        // Verify the zip file was created
        assert!(Path::new(&zip_file_path).exists());

        // Clean up
        fs::remove_file(zip_file_path).expect("Failed to delete zip file");
    }

    #[tokio::test]
    async fn test_write_webp_to_folder() {
        let images = vec![
            create_test_image(100, 100, 255),
            create_test_image(200, 200, 128),
        ];

        let result = file_manager::write_webps_to_folder(&images).await;

        assert!(result.is_ok());
        let dir_path = result.unwrap();

        // Verify the directory was created
        assert!(Path::new(&dir_path).exists());

        // Check if the expected WebP files are created
        for i in 0..images.len() {
            let file_path = format!("{}/slice_{:04}.webp", dir_path, i);
            assert!(Path::new(&file_path).exists());
        }

        // Clean up
        fs::remove_dir_all(dir_path).expect("Failed to delete directory");
    }

    #[test]
    fn test_convert_luma_to_rgb() {
        let luma_image = create_test_image(2, 2, 100); // 2x2 image with Luma value 100

        let rgb_image = file_manager::convert_luma_to_rgb(&luma_image);

        assert_eq!(rgb_image.width(), 2);
        assert_eq!(rgb_image.height(), 2);

        for (_, _, pixel) in rgb_image.enumerate_pixels() {
            assert_eq!(pixel.0[0], 100);
            assert_eq!(pixel.0[1], 100);
            assert_eq!(pixel.0[2], 100);
        }
    }
}
