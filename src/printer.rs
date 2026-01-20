use std::{fs, io::Write, path::Path};

use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Printer {
    pub name: String,
    pub brand: String,
    pub model: String,
    pub physical_x: f64, // millimeters
    pub physical_y: f64, // millimeters
    pub physical_z: f64, // millimeters
    pub pixel_x: u32,
    pub pixel_y: u32,
}

impl Default for Printer {
    fn default() -> Self {
        Printer::load_from_file(Path::new("config/printers/debug/debug.toml")).unwrap()
    }
}

impl Printer {
    pub fn load_from_file(path: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        let content = fs::read_to_string(path)?;
        let settings: Printer = toml::from_str(&content)?;
        Ok(settings)
    }

    pub fn save_to_file(&self, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        let content = toml::to_string(self)?;
        let mut file = fs::File::create(path)?;
        file.write_all(content.as_bytes())?;
        Ok(())
    }
}
