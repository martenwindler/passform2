use serde::de::DeserializeOwned;
use std::fs::File;
use std::path::Path;

pub mod helper {
    /// Wandelt eine UUID in einen ROS-konformen Namen um (module_uuid_mit_unterstrichen)
    pub fn ros_name(uuid: &str) -> String {
        format!("module_{}", uuid.replace('-', "_"))
    }

    /// Revertiert den ROS-Namen zurück zur originalen UUID
    pub fn from_ros_name(ros_name: &str) -> String {
        ros_name
            .replace("module_", "")
            .replace('_', "-")
    }

    /// Universeller YAML-Loader für Rust-Strukturen (ersetzt load_yaml_file)
    pub fn load_yaml<T: DeserializeOwned>(path: &Path) -> Result<T, Box<dyn std::error::Error>> {
        let file = File::open(path)?;
        let decoded: T = serde_yaml::from_reader(file)?;
        Ok(decoded)
    }
}