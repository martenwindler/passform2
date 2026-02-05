use serde::de::DeserializeOwned;
use std::fs::File;
use std::path::Path;
use serde::{Serialize, Deserialize}; // Falls du das Enum über das Socket senden willst

/// Bereinigt eine ID für BaSyx (nur alphanumerisch und Unterstriche)
pub fn sanitize_id(id: &str) -> String {
    id.replace(|c: char| !c.is_alphanumeric() && c != '_', "_")
}

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

/// Universeller YAML-Loader für Rust-Strukturen
pub fn load_yaml<T: DeserializeOwned>(path: &Path) -> Result<T, Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    let decoded: T = serde_yaml::from_reader(file)?;
    Ok(decoded)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[repr(i32)] 
pub enum LogLevel {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
    Fatal = 4,
}

// Erlaubt: LogLevel::Info as i32
impl From<i32> for LogLevel {
    fn from(value: i32) -> Self {
        match value {
            0 => LogLevel::Debug,
            1 => LogLevel::Info,
            2 => LogLevel::Warn,
            3 => LogLevel::Error,
            _ => LogLevel::Fatal,
        }
    }
}