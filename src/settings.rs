use crate::SharedSettings; // Ensure this is correctly defined as Arc<Mutex<Settings>> or similar
use dirs_next::config_dir; // Use dirs-next for better maintenance
use serde::{Deserialize, Serialize};
use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};
use thiserror::Error; // For better error handling

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct GeneralSettings {
    pub username: String,
    pub theme: String,
    pub auto_save: bool,
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct RendererSettings {
    pub render_scale: f32,
    pub visualize_edges: bool,
    pub visualize_normals: bool,
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct NetworkSettings {
    pub timeout: u32,
    pub use_https: bool,
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Settings {
    pub general: GeneralSettings,
    pub renderer: RendererSettings,
    pub network: NetworkSettings,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            general: GeneralSettings {
                username: String::from("Egg"),
                theme: String::from("system"),
                auto_save: true,
            },
            renderer: RendererSettings {
                render_scale: 1.0,
                visualize_edges: true,
                visualize_normals: false,
            },
            network: NetworkSettings {
                timeout: 30,
                use_https: true,
            },
        }
    }
}

#[derive(Debug, Error)]
pub enum SettingsError {
    #[error("IO Error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Serialization/Deserialization Error: {0}")]
    Serde(#[from] toml::de::Error),

    #[error("Serialization Error: {0}")]
    SerdeSer(#[from] toml::ser::Error),

    #[error("Configuration Directory Not Found")]
    ConfigDirNotFound,
}

impl Settings {
    /// Retrieves the path to the user settings file.
    fn user_settings_path() -> Result<PathBuf, SettingsError> {
        let config_dir = config_dir().ok_or(SettingsError::ConfigDirNotFound)?;
        Ok(config_dir.join("SealSlicer")            .join("settings")
        .join("user_settings.toml"))
    }

    /// Retrieves the path to the default settings file.
    fn default_settings_path() -> Result<PathBuf, SettingsError> {
        let config_dir = config_dir().ok_or(SettingsError::ConfigDirNotFound)?;
        Ok(config_dir
            .join("SealSlicer")
            .join("settings")
            .join("default_settings.toml"))
    }

    /// Loads settings from a specified file path.
    pub fn load_from_file(path: &Path) -> Result<Self, SettingsError> {
        let content = fs::read_to_string(path)?;
        let settings: Settings = toml::from_str(&content)?;
        Ok(settings)
    }

    /// Saves settings to a specified file path, ensuring the directory exists.
    pub fn save_to_file(&self, path: &Path) -> Result<(), SettingsError> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }

        let content = toml::to_string_pretty(self)?;
        fs::write(path, content)?;
        Ok(())
    }

    /// Saves settings to the user settings file.
    pub fn save_user_settings(&self) -> Result<(), SettingsError> {
        let user_path = Settings::user_settings_path()?;
        self.save_to_file(&user_path)
    }

    /// Loads user settings, handling defaults and creating necessary files.
    pub fn load_user_settings() -> SharedSettings {
        match Settings::initialize_settings() {
            Ok(settings) => Arc::new(Mutex::new(settings)),
            Err(e) => {
                eprintln!("Error initializing settings: {}", e);
                Arc::new(Mutex::new(Settings::default()))
            }
        }
    }

    /// Initializes settings by loading user settings or falling back to defaults.
    fn initialize_settings() -> Result<Self, SettingsError> {
        let user_settings_path = Settings::user_settings_path()?;
        let default_settings_path = Settings::default_settings_path()?;

        if user_settings_path.exists() {
            // Attempt to load user settings
            match Settings::load_from_file(&user_settings_path) {
                Ok(settings) => Ok(settings),
                Err(e) => {
                    eprintln!(
                        "Failed to load user settings: {}. Attempting to load defaults.",
                        e
                    );
                    Settings::load_defaults(user_settings_path, default_settings_path)
                }
            }
        } else {
            // User settings do not exist; attempt to load defaults
            println!("User settings not found. Loading default settings.");
            Settings::load_defaults(user_settings_path, default_settings_path)
        }
    }

    /// Loads default settings and saves them as user settings.
    fn load_defaults(user_path: PathBuf, default_path: PathBuf) -> Result<Self, SettingsError> {
        if default_path.exists() {
            let settings = Settings::load_from_file(&default_path)?;
            // Save default settings as user settings
            settings.save_to_file(&user_path)?;
            println!("Default settings loaded and saved as user settings.");
            Ok(settings)
        } else {
            eprintln!("Default settings file not found. Using hardcoded defaults.");
            let settings = Settings::default();
            // Save hardcoded defaults as default and user settings
            settings.save_to_file(&default_path)?;
            settings.save_to_file(&user_path)?;
            println!("Hardcoded defaults saved as default and user settings.");
            Ok(settings)
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::env;
    use std::fs;
    use std::path::PathBuf;
    use tempfile::tempdir;
    use serial_test::serial; // To run tests serially when modifying environment variables

    /// Helper function to set up a temporary configuration directory
    fn setup_temp_config_dir() -> (tempfile::TempDir, PathBuf) {
        let temp_dir = tempdir().expect("Failed to create temporary directory");
        let config_dir = temp_dir.path().to_path_buf();
        (temp_dir, config_dir)
    }

    /// Helper function to override the config_dir() function from dirs-next
    /// by temporarily setting the HOME and XDG_CONFIG_HOME environment variables to the temp config directory
    fn override_config_dir(temp_config_dir: &PathBuf) {
        env::set_var("HOME", temp_config_dir);
        env::set_var("XDG_CONFIG_HOME", temp_config_dir);
    }

    /// Helper function to reset the HOME and XDG_CONFIG_HOME environment variables after tests
    fn reset_config_dir(original_home: &str, original_xdg_config_home: Option<&str>) {
        env::set_var("HOME", original_home);
        match original_xdg_config_home {
            Some(val) => env::set_var("XDG_CONFIG_HOME", val),
            None => env::remove_var("XDG_CONFIG_HOME"),
        }
    }

    /// Test Case 1a: Verify that `user_settings_path()` correctly constructs the path
    #[test]
    #[serial]
    fn test_user_settings_path_retrieval() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        let expected_path = config_dir.join("SealSlicer").join("settings").join("user_settings.toml");
        let actual_path = Settings::user_settings_path().expect("Failed to get user settings path");

        assert_eq!(actual_path, expected_path);

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
        // TempDir is automatically deleted when it goes out of scope
    }

    /// Test Case 1b: Verify that `default_settings_path()` correctly constructs the path
    #[test]
    #[serial]
    fn test_default_settings_path_retrieval() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        let expected_path = config_dir
            .join("SealSlicer")
            .join("settings")
            .join("default_settings.toml");
        let actual_path =
            Settings::default_settings_path().expect("Failed to get default settings path");

        assert_eq!(actual_path, expected_path);

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 1c: Handling Non-Existent Configuration Directory
    ///
    /// NOTE: This test is marked as ignored because fully simulating the absence of a configuration directory
    /// is complex due to dirs-next's fallback mechanisms. Consider refactoring your code to support dependency
    /// injection for more robust testing.
    #[test]
    #[serial]
    #[ignore]
    fn test_config_dir_not_found() {
        // Temporarily unset the HOME and XDG_CONFIG_HOME environment variables
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        env::remove_var("HOME");
        env::remove_var("XDG_CONFIG_HOME");

        let result = Settings::user_settings_path();
        assert!(matches!(result, Err(SettingsError::ConfigDirNotFound)));

        let result = Settings::default_settings_path();
        assert!(matches!(result, Err(SettingsError::ConfigDirNotFound)));

        // Restore the HOME and XDG_CONFIG_HOME environment variables
        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 2a: Loading from a Valid User Settings File
    #[test]
    #[serial]
    fn test_load_from_valid_user_settings_file() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create user_settings.toml with specific content
        let user_settings_path = Settings::user_settings_path().unwrap();
        fs::create_dir_all(user_settings_path.parent().unwrap()).unwrap();
        let user_settings_content = r#"
            [general]
            username = "TestUser"
            theme = "dark"
            auto_save = false

            [renderer]
            render_scale = 2.5
            visualize_edges = false
            visualize_normals = true

            [network]
            timeout = 60
            use_https = false
        "#;
        fs::write(&user_settings_path, user_settings_content).unwrap();

        // Load settings
        let settings = Settings::load_from_file(&user_settings_path).unwrap();

        // Assert values
        assert_eq!(settings.general.username, "TestUser");
        assert_eq!(settings.general.theme, "dark");
        assert_eq!(settings.general.auto_save, false);

        assert_eq!(settings.renderer.render_scale, 2.5);
        assert_eq!(settings.renderer.visualize_edges, false);
        assert_eq!(settings.renderer.visualize_normals, true);

        assert_eq!(settings.network.timeout, 60);
        assert_eq!(settings.network.use_https, false);

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 2b: Loading from a Valid Default Settings File
    #[test]
    #[serial]
    fn test_load_from_valid_default_settings_file() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create default_settings.toml with specific content
        let default_settings_path = Settings::default_settings_path().unwrap();
        fs::create_dir_all(default_settings_path.parent().unwrap()).unwrap();
        let default_settings_content = r#"
            [general]
            username = "DefaultUser"
            theme = "light"
            auto_save = true

            [renderer]
            render_scale = 1.5
            visualize_edges = true
            visualize_normals = false

            [network]
            timeout = 45
            use_https = true
        "#;
        fs::write(&default_settings_path, default_settings_content).unwrap();

        // Load settings
        let settings = Settings::load_from_file(&default_settings_path).unwrap();

        // Assert values
        assert_eq!(settings.general.username, "DefaultUser");
        assert_eq!(settings.general.theme, "light");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 1.5);
        assert_eq!(settings.renderer.visualize_edges, true);
        assert_eq!(settings.renderer.visualize_normals, false);

        assert_eq!(settings.network.timeout, 45);
        assert_eq!(settings.network.use_https, true);

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 2c: Loading When User Settings File is Missing but Default Exists
    #[test]
    #[serial]
    fn test_load_user_settings_missing_user_but_default_exists() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create default_settings.toml
        let default_settings_path = Settings::default_settings_path().unwrap();
        fs::create_dir_all(default_settings_path.parent().unwrap()).unwrap();
        let default_settings_content = r#"
            [general]
            username = "DefaultUser"
            theme = "light"
            auto_save = true

            [renderer]
            render_scale = 1.5
            visualize_edges = true
            visualize_normals = false

            [network]
            timeout = 45
            use_https = true
        "#;
        fs::write(&default_settings_path, default_settings_content).unwrap();

        // Ensure user_settings.toml does not exist
        let user_settings_path = Settings::user_settings_path().unwrap();
        assert!(!user_settings_path.exists());

        // Initialize settings
        let settings = Settings::initialize_settings().unwrap();

        // Assert loaded settings match default
        assert_eq!(settings.general.username, "DefaultUser");
        assert_eq!(settings.general.theme, "light");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 1.5);
        assert_eq!(settings.renderer.visualize_edges, true);
        assert_eq!(settings.renderer.visualize_normals, false);

        assert_eq!(settings.network.timeout, 45);
        assert_eq!(settings.network.use_https, true);

        // Assert that user_settings.toml is created with default settings
        assert!(user_settings_path.exists());

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 2d: Loading When Both User and Default Settings Files are Missing
    #[test]
    #[serial]
    fn test_load_user_settings_missing_both_files() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Ensure both user_settings.toml and default_settings.toml do not exist
        let user_settings_path = Settings::user_settings_path().unwrap();
        let default_settings_path = Settings::default_settings_path().unwrap();
        assert!(!user_settings_path.exists());
        assert!(!default_settings_path.exists());

        // Initialize settings
        let settings = Settings::initialize_settings().unwrap();

        // Assert loaded settings match hardcoded defaults
        assert_eq!(settings.general.username, "Egg");
        assert_eq!(settings.general.theme, "system");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 1.0);
        assert_eq!(settings.renderer.visualize_edges, true);
        assert_eq!(settings.renderer.visualize_normals, false);

        assert_eq!(settings.network.timeout, 30);
        assert_eq!(settings.network.use_https, true);

        // Assert that both default_settings.toml and user_settings.toml are created
        assert!(default_settings_path.exists());
        assert!(user_settings_path.exists());

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 2e: Handling Corrupted Settings Files
    #[test]
    #[serial]
    fn test_handling_corrupted_settings_files() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create a corrupted user_settings.toml
        let user_settings_path = Settings::user_settings_path().unwrap();
        fs::create_dir_all(user_settings_path.parent().unwrap()).unwrap();
        let corrupted_content = "invalid toml content ::::";
        fs::write(&user_settings_path, corrupted_content).unwrap();

        // Create a valid default_settings.toml
        let default_settings_path = Settings::default_settings_path().unwrap();
        fs::create_dir_all(default_settings_path.parent().unwrap()).unwrap();
        let default_settings_content = r#"
            [general]
            username = "DefaultUser"
            theme = "light"
            auto_save = true

            [renderer]
            render_scale = 1.5
            visualize_edges = true
            visualize_normals = false

            [network]
            timeout = 45
            use_https = true
        "#;
        fs::write(&default_settings_path, default_settings_content).unwrap();

        // Initialize settings, which should fallback to defaults
        let settings = Settings::initialize_settings().unwrap();

        // Assert loaded settings match default
        assert_eq!(settings.general.username, "DefaultUser");
        assert_eq!(settings.general.theme, "light");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 1.5);
        assert_eq!(settings.renderer.visualize_edges, true);
        assert_eq!(settings.renderer.visualize_normals, false);

        assert_eq!(settings.network.timeout, 45);
        assert_eq!(settings.network.use_https, true);

        // Assert that user_settings.toml is overwritten with default settings
        let loaded_user_settings = Settings::load_from_file(&user_settings_path).unwrap();
        assert_eq!(loaded_user_settings.general.username, "DefaultUser");

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 3a: Successfully Saving User Settings
    #[test]
    #[serial]
    fn test_save_user_settings_success() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create a Settings instance with custom values
        let settings = Settings {
            general: GeneralSettings {
                username: "CustomUser".to_string(),
                theme: "dark".to_string(),
                auto_save: false,
            },
            renderer: RendererSettings {
                render_scale: 2.0,
                visualize_edges: false,
                visualize_normals: true,
            },
            network: NetworkSettings {
                timeout: 50,
                use_https: false,
            },
        };

        // Save user settings
        settings.save_user_settings().unwrap();

        // Read the saved file and verify contents
        let user_settings_path = Settings::user_settings_path().unwrap();
        let saved_content = fs::read_to_string(&user_settings_path).unwrap();
        let loaded_settings: Settings = toml::from_str(&saved_content).unwrap();

        assert_eq!(loaded_settings.general.username, "CustomUser");
        assert_eq!(loaded_settings.general.theme, "dark");
        assert_eq!(loaded_settings.general.auto_save, false);

        assert_eq!(loaded_settings.renderer.render_scale, 2.0);
        assert_eq!(loaded_settings.renderer.visualize_edges, false);
        assert_eq!(loaded_settings.renderer.visualize_normals, true);

        assert_eq!(loaded_settings.network.timeout, 50);
        assert_eq!(loaded_settings.network.use_https, false);

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 3b: Successfully Saving Default Settings
    #[test]
    #[serial]
    fn test_save_default_settings_success() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create a Settings instance with custom default values
        let settings = Settings {
            general: GeneralSettings {
                username: "DefaultUser".to_string(),
                theme: "light".to_string(),
                auto_save: true,
            },
            renderer: RendererSettings {
                render_scale: 1.2,
                visualize_edges: true,
                visualize_normals: false,
            },
            network: NetworkSettings {
                timeout: 40,
                use_https: true,
            },
        };

        // Save default settings
        let default_settings_path = Settings::default_settings_path().unwrap();
        settings.save_to_file(&default_settings_path).unwrap();

        // Read the saved default file and verify contents
        let saved_content = fs::read_to_string(&default_settings_path).unwrap();
        let loaded_settings: Settings = toml::from_str(&saved_content).unwrap();

        assert_eq!(loaded_settings.general.username, "DefaultUser");
        assert_eq!(loaded_settings.general.theme, "light");
        assert_eq!(loaded_settings.general.auto_save, true);

        assert_eq!(loaded_settings.renderer.render_scale, 1.2);
        assert_eq!(loaded_settings.renderer.visualize_edges, true);
        assert_eq!(loaded_settings.renderer.visualize_normals, false);

        assert_eq!(loaded_settings.network.timeout, 40);
        assert_eq!(loaded_settings.network.use_https, true);

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 3c: Handling Directory Creation During Save
    #[test]
    #[serial]
    fn test_save_creates_directories() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Define a non-existent directory path within the config directory
        let custom_settings_path = config_dir.join("SealSlicer").join("settings").join("custom_settings.toml");

        // Create a Settings instance
        let settings = Settings::default();

        // Save to the custom path, which requires directory creation
        settings.save_to_file(&custom_settings_path).unwrap();

        // Assert that the file exists
        assert!(custom_settings_path.exists());

        // Optionally, verify the contents match the default settings
        let saved_content = fs::read_to_string(&custom_settings_path).unwrap();
        let loaded_settings: Settings = toml::from_str(&saved_content).unwrap();

        assert_eq!(loaded_settings, Settings::default());

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 3d: Handling File Write Permissions
    #[test]
    #[serial]
    fn test_save_file_write_permissions() {
        use std::os::unix::fs::PermissionsExt;

        // This test is Unix-specific due to permissions handling
        #[cfg(unix)]
        {
            let (_temp_dir, config_dir) = setup_temp_config_dir();
            let original_home = env::var("HOME").unwrap_or_default();
            let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
            override_config_dir(&config_dir);

            // Create a directory with no write permissions
            let no_write_dir = config_dir.join("SealSlicer").join("no_write_dir");
            fs::create_dir_all(&no_write_dir).unwrap();

            // Set the directory permissions to read-only
            fs::set_permissions(&no_write_dir, fs::Permissions::from_mode(0o555)).unwrap();

            // Attempt to save a settings file within the read-only directory
            let settings = Settings::default();
            let read_only_settings_path = no_write_dir.join("user_settings.toml");

            let result = settings.save_to_file(&read_only_settings_path);
            assert!(matches!(result, Err(SettingsError::Io(_))));

            reset_config_dir(&original_home, original_xdg_config_home.as_deref());
        }

        // On non-Unix systems, skip the test
        #[cfg(not(unix))]
        {
            todo!("guh");
        }
    }

    /// Test Case 4a: Successful Initialization with Existing User Settings
    #[test]
    #[serial]
    fn test_initialization_with_existing_user_settings() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create user_settings.toml with specific content
        let user_settings_path = Settings::user_settings_path().unwrap();
        fs::create_dir_all(user_settings_path.parent().unwrap()).unwrap();
        let user_settings_content = r#"
            [general]
            username = "ExistingUser"
            theme = "dark"
            auto_save = false

            [renderer]
            render_scale = 2.0
            visualize_edges = false
            visualize_normals = true

            [network]
            timeout = 60
            use_https = false
        "#;
        fs::write(&user_settings_path, user_settings_content).unwrap();

        // Initialize settings
        let settings = Settings::initialize_settings().unwrap();

        // Assert loaded settings match user settings
        assert_eq!(settings.general.username, "ExistingUser");
        assert_eq!(settings.general.theme, "dark");
        assert_eq!(settings.general.auto_save, false);

        assert_eq!(settings.renderer.render_scale, 2.0);
        assert_eq!(settings.renderer.visualize_edges, false);
        assert_eq!(settings.renderer.visualize_normals, true);

        assert_eq!(settings.network.timeout, 60);
        assert_eq!(settings.network.use_https, false);

        // Ensure default_settings.toml is not modified
        let default_settings_path = Settings::default_settings_path().unwrap();
        assert!(!default_settings_path.exists());

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 4b: Initialization with Corrupted User Settings Falling Back to Defaults
    #[test]
    #[serial]
    fn test_initialization_with_corrupted_user_settings_fallback_to_defaults() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create corrupted user_settings.toml
        let user_settings_path = Settings::user_settings_path().unwrap();
        fs::create_dir_all(user_settings_path.parent().unwrap()).unwrap();
        fs::write(&user_settings_path, "corrupted content").unwrap();

        // Create a valid default_settings.toml
        let default_settings_path = Settings::default_settings_path().unwrap();
        fs::create_dir_all(default_settings_path.parent().unwrap()).unwrap();
        let default_settings_content = r#"
            [general]
            username = "DefaultUser"
            theme = "light"
            auto_save = true

            [renderer]
            render_scale = 1.5
            visualize_edges = true
            visualize_normals = false

            [network]
            timeout = 45
            use_https = true
        "#;
        fs::write(&default_settings_path, default_settings_content).unwrap();

        // Initialize settings, which should fallback to defaults
        let settings = Settings::initialize_settings().unwrap();

        // Assert loaded settings match default
        assert_eq!(settings.general.username, "DefaultUser");
        assert_eq!(settings.general.theme, "light");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 1.5);
        assert_eq!(settings.renderer.visualize_edges, true);
        assert_eq!(settings.renderer.visualize_normals, false);

        assert_eq!(settings.network.timeout, 45);
        assert_eq!(settings.network.use_https, true);

        // Assert that user_settings.toml is overwritten with default settings
        let loaded_user_settings = Settings::load_from_file(&user_settings_path).unwrap();
        assert_eq!(loaded_user_settings.general.username, "DefaultUser");

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 4c: Initialization with Missing User and Default Settings
    #[test]
    #[serial]
    fn test_initialization_with_missing_user_and_default_settings() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Ensure both user_settings.toml and default_settings.toml do not exist
        let user_settings_path = Settings::user_settings_path().unwrap();
        let default_settings_path = Settings::default_settings_path().unwrap();
        assert!(!user_settings_path.exists());
        assert!(!default_settings_path.exists());

        // Initialize settings
        let settings = Settings::initialize_settings().unwrap();

        // Assert loaded settings match hardcoded defaults
        assert_eq!(settings.general.username, "Egg");
        assert_eq!(settings.general.theme, "system");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 1.0);
        assert_eq!(settings.renderer.visualize_edges, true);
        assert_eq!(settings.renderer.visualize_normals, false);

        assert_eq!(settings.network.timeout, 30);
        assert_eq!(settings.network.use_https, true);

        // Assert that both default_settings.toml and user_settings.toml are created
        assert!(default_settings_path.exists());
        assert!(user_settings_path.exists());

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 5a: Correct Serialization of Settings
    #[test]
    fn test_correct_serialization_of_settings() {
        let settings = Settings {
            general: GeneralSettings {
                username: "SerializeUser".to_string(),
                theme: "blue".to_string(),
                auto_save: false,
            },
            renderer: RendererSettings {
                render_scale: 3.0,
                visualize_edges: true,
                visualize_normals: true,
            },
            network: NetworkSettings {
                timeout: 100,
                use_https: false,
            },
        };

        let serialized = toml::to_string_pretty(&settings).unwrap();
        let expected = r#"
[general]
username = "SerializeUser"
theme = "blue"
auto_save = false

[renderer]
render_scale = 3.0
visualize_edges = true
visualize_normals = true

[network]
timeout = 100
use_https = false
"#.trim();

        assert_eq!(serialized.trim(), expected);
    }

    /// Test Case 5b: Correct Deserialization of TOML to Settings
    #[test]
    fn test_correct_deserialization_of_toml_to_settings() {
        let toml_content = r#"
            [general]
            username = "DeserializeUser"
            theme = "green"
            auto_save = true

            [renderer]
            render_scale = 4.5
            visualize_edges = false
            visualize_normals = true

            [network]
            timeout = 120
            use_https = true
        "#;

        let settings: Settings = toml::from_str(toml_content).unwrap();

        assert_eq!(settings.general.username, "DeserializeUser");
        assert_eq!(settings.general.theme, "green");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 4.5);
        assert_eq!(settings.renderer.visualize_edges, false);
        assert_eq!(settings.renderer.visualize_normals, true);

        assert_eq!(settings.network.timeout, 120);
        assert_eq!(settings.network.use_https, true);
    }

    /// Test Case 5c: Handling Missing Fields During Deserialization
    #[test]
    fn test_handling_missing_fields_during_deserialization() {
        // TOML content missing the [network] section
        let toml_content = r#"
            [general]
            username = "PartialUser"
            theme = "red"
            auto_save = true

            [renderer]
            render_scale = 2.5
            visualize_edges = true
            visualize_normals = false
        "#;

        let result: Result<Settings, toml::de::Error> = toml::from_str(toml_content);
        assert!(result.is_err());
    }

    /// Test Case 6a: Correct Default Values
    #[test]
    fn test_correct_default_values() {
        let default_settings = Settings::default();

        assert_eq!(default_settings.general.username, "Egg");
        assert_eq!(default_settings.general.theme, "system");
        assert_eq!(default_settings.general.auto_save, true);

        assert_eq!(default_settings.renderer.render_scale, 1.0);
        assert_eq!(default_settings.renderer.visualize_edges, true);
        assert_eq!(default_settings.renderer.visualize_normals, false);

        assert_eq!(default_settings.network.timeout, 30);
        assert_eq!(default_settings.network.use_https, true);
    }

    /// Test Case 6b: Overriding Defaults When Loading from Files
    #[test]
    #[serial]
    fn test_overriding_defaults_when_loading_from_files() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create default_settings.toml with specific values
        let default_settings_path = Settings::default_settings_path().unwrap();
        fs::create_dir_all(default_settings_path.parent().unwrap()).unwrap();
        let default_settings_content = r#"
            [general]
            username = "OverriddenDefaultUser"
            theme = "yellow"
            auto_save = false

            [renderer]
            render_scale = 2.2
            visualize_edges = false
            visualize_normals = true

            [network]
            timeout = 55
            use_https = false
        "#;
        fs::write(&default_settings_path, default_settings_content).unwrap();

        // Create user_settings.toml with different values
        let user_settings_path = Settings::user_settings_path().unwrap();
        let user_settings_content = r#"
            [general]
            username = "UserOverride"
            theme = "purple"
            auto_save = true

            [renderer]
            render_scale = 3.3
            visualize_edges = true
            visualize_normals = false

            [network]
            timeout = 75
            use_https = true
        "#;
        fs::write(&user_settings_path, user_settings_content).unwrap();

        // Initialize settings
        let settings = Settings::initialize_settings().unwrap();

        // Assert that settings match user_settings.toml, not default_settings.toml
        assert_eq!(settings.general.username, "UserOverride");
        assert_eq!(settings.general.theme, "purple");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 3.3);
        assert_eq!(settings.renderer.visualize_edges, true);
        assert_eq!(settings.renderer.visualize_normals, false);

        assert_eq!(settings.network.timeout, 75);
        assert_eq!(settings.network.use_https, true);

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 7a: Proper Error Propagation
    #[test]
    #[serial]
    fn test_proper_error_propagation() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Attempt to load settings from a non-existent path
        let non_existent_path = config_dir.join("SealSlicer").join("non_existent.toml");
        let result = Settings::load_from_file(&non_existent_path);
        assert!(matches!(result, Err(SettingsError::Io(_))));

        // Attempt to deserialize invalid TOML
        let invalid_toml = "invalid = toml:::";
        let invalid_path = config_dir.join("SealSlicer").join("invalid.toml");
        fs::create_dir_all(invalid_path.parent().unwrap()).unwrap();
        fs::write(&invalid_path, invalid_toml).unwrap();
        let result = Settings::load_from_file(&invalid_path);
        assert!(matches!(result, Err(SettingsError::Serde(_))));

        // Attempt to serialize to an invalid path (e.g., directory instead of file)
        let invalid_save_path = config_dir.join("SealSlicer").join("settings");
        fs::create_dir_all(&invalid_save_path).unwrap();
        let settings = Settings::default();
        let save_result = settings.save_to_file(&invalid_save_path);
        assert!(matches!(save_result, Err(SettingsError::Io(_))));

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 7b: Logging of Errors and Informational Messages
    #[test]
    #[serial]
    fn test_logging_of_errors_and_info_messages() {
        // Note: Capturing printed output in Rust tests is non-trivial and often not recommended.
        // Instead, consider abstracting logging to a separate component that can be mocked or
        // injected during tests. Here, we'll ensure that functions return appropriate errors
        // which can be asserted.

        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create a corrupted user_settings.toml
        let user_settings_path = Settings::user_settings_path().unwrap();
        fs::create_dir_all(user_settings_path.parent().unwrap()).unwrap();
        fs::write(&user_settings_path, "corrupted content").unwrap();

        // Create a valid default_settings.toml
        let default_settings_path = Settings::default_settings_path().unwrap();
        fs::create_dir_all(default_settings_path.parent().unwrap()).unwrap();
        let default_settings_content = r#"
            [general]
            username = "DefaultUser"
            theme = "light"
            auto_save = true

            [renderer]
            render_scale = 1.5
            visualize_edges = true
            visualize_normals = false

            [network]
            timeout = 45
            use_https = true
        "#;
        fs::write(&default_settings_path, default_settings_content).unwrap();

        // Initialize settings, expecting fallback to defaults
        let settings_result = Settings::initialize_settings();
        assert!(settings_result.is_ok());
        let settings = settings_result.unwrap();

        // Verify that settings match defaults
        assert_eq!(settings.general.username, "DefaultUser");
        assert_eq!(settings.general.theme, "light");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 1.5);
        assert_eq!(settings.renderer.visualize_edges, true);
        assert_eq!(settings.renderer.visualize_normals, false);

        assert_eq!(settings.network.timeout, 45);
        assert_eq!(settings.network.use_https, true);

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 8a: Concurrent Access to SharedSettings
    #[test]
    fn test_concurrent_access_to_shared_settings() {
        use std::sync::Arc;
        use std::thread;

        let settings = Settings::default();
        let shared_settings: SharedSettings = Arc::new(Mutex::new(settings));

        let mut handles = vec![];

        for i in 0..10 {
            let shared_clone = Arc::clone(&shared_settings);
            handles.push(thread::spawn(move || {
                let mut settings = shared_clone.lock().unwrap();
                settings.general.username = format!("User{}", i);
                settings.network.timeout += i;
            }));
        }

        for handle in handles {
            handle.join().unwrap();
        }

        let final_settings = shared_settings.lock().unwrap();
        // The final username and timeout are nondeterministic since threads run concurrently
        // But we can assert that timeout has been incremented correctly
        // Initial timeout is 30, added by 0+1+2+...+9 = 45, so final timeout should be 75
        assert_eq!(final_settings.network.timeout, 75);
    }

    /// Test Case 8b: Deadlock Prevention
    #[test]
    fn test_deadlock_prevention() {
        use std::sync::Arc;
        use std::thread;

        let settings = Settings::default();
        let shared_settings: SharedSettings = Arc::new(Mutex::new(settings));

        let shared_clone1 = Arc::clone(&shared_settings);
        let shared_clone2 = Arc::clone(&shared_settings);

        let handle1 = thread::spawn(move || {
            let mut settings = shared_clone1.lock().unwrap();
            settings.general.username = "Thread1".to_string();
        });

        let handle2 = thread::spawn(move || {
            let mut settings = shared_clone2.lock().unwrap();
            settings.network.timeout = 100;
        });

        handle1.join().unwrap();
        handle2.join().unwrap();

        let final_settings = shared_settings.lock().unwrap();
        assert_eq!(final_settings.general.username, "Thread1");
        assert_eq!(final_settings.network.timeout, 100);
    }

    
    /// Test Case 10a: Full Load and Save Cycle (Integration Test)
    #[test]
    #[serial]
    fn test_full_load_and_save_cycle() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Initialize settings (should create defaults)
        let mut settings = Settings::initialize_settings().unwrap();

        // Modify a setting
        
            settings.general.username = "CycleUser".to_string();
            settings.network.timeout = 90;
        

        // Save settings
        settings.save_user_settings().unwrap();

        // Reload settings
        let reloaded_settings = Settings::load_from_file(&Settings::user_settings_path().unwrap()).unwrap();

        // Assert that changes persist
        assert_eq!(reloaded_settings.general.username, "CycleUser");
        assert_eq!(reloaded_settings.network.timeout, 90);

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }

    /// Test Case 10b: Recovery from Corrupted User Settings
    #[test]
    #[serial]
    fn test_recovery_from_corrupted_user_settings() {
        let (_temp_dir, config_dir) = setup_temp_config_dir();
        let original_home = env::var("HOME").unwrap_or_default();
        let original_xdg_config_home = env::var("XDG_CONFIG_HOME").ok();
        override_config_dir(&config_dir);

        // Create corrupted user_settings.toml
        let user_settings_path = Settings::user_settings_path().unwrap();
        fs::create_dir_all(user_settings_path.parent().unwrap()).unwrap();
        fs::write(&user_settings_path, "corrupted content").unwrap();

        // Create a valid default_settings.toml
        let default_settings_path = Settings::default_settings_path().unwrap();
        fs::create_dir_all(default_settings_path.parent().unwrap()).unwrap();
        let default_settings_content = r#"
            [general]
            username = "DefaultUser"
            theme = "light"
            auto_save = true

            [renderer]
            render_scale = 1.5
            visualize_edges = true
            visualize_normals = false

            [network]
            timeout = 45
            use_https = true
        "#;
        fs::write(&default_settings_path, default_settings_content).unwrap();

        // Initialize settings, expecting fallback to defaults
        let settings = Settings::initialize_settings().unwrap();

        // Assert that settings match defaults
        assert_eq!(settings.general.username, "DefaultUser");
        assert_eq!(settings.general.theme, "light");
        assert_eq!(settings.general.auto_save, true);

        assert_eq!(settings.renderer.render_scale, 1.5);
        assert_eq!(settings.renderer.visualize_edges, true);
        assert_eq!(settings.renderer.visualize_normals, false);

        assert_eq!(settings.network.timeout, 45);
        assert_eq!(settings.network.use_https, true);

        // Assert that user_settings.toml is overwritten with default settings
        let loaded_user_settings = Settings::load_from_file(&user_settings_path).unwrap();
        assert_eq!(loaded_user_settings.general.username, "DefaultUser");

        reset_config_dir(&original_home, original_xdg_config_home.as_deref());
    }
}
