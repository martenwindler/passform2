use std::fs;
use std::path::PathBuf;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use tracing::{info, error, warn};
use chrono::Local;

// --- DATENSTRUKTUR FÖR DE SSOT ---

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SSoTData {
    pub config: Value,
    pub agents: Vec<Value>,
    pub last_update: Option<String>,
}

impl Default for SSoTData {
    fn default() -> Self {
        Self {
            config: json!({"grid": {"width": 10, "height": 10}}),
            agents: vec![],
            last_update: None,
        }
    }
}

// --- MANAGER ---

pub struct ConfigManager {
    pub file_path: PathBuf,
}

impl ConfigManager {
    pub fn new() -> Self {
        // Dynamische Pfadfindung: Wi kiekt, wo dat Executable liggt oder bruukt dat aktuelle Verzeichnis
        // In Rust is dat sekerer, den Pfad relativ to de Cargo-Root oder Executable to setten
        let mut path = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
        path.push("data");
        path.push("config.json");

        let manager = Self { file_path: path };
        manager.ensure_data_dir();
        manager
    }

    fn ensure_data_dir(&self) {
        if let Some(parent) = self.file_path.parent() {
            if !parent.exists() {
                info!("📁 Erstelle Daten-Verzeichnis: {:?}", parent);
                fs::create_dir_all(parent).expect("Kunn dat Verzeichnis nich erstellen!");
            }
        }
    }

    /// Lädt de SSoT Datei (Ersatz för load_config)
    pub fn load_config(&self) -> SSoTData {
        if !self.file_path.exists() {
            warn!("⚠️ SSoT Datei nich funnen ({:?}), nimm Defaults.", self.file_path);
            return SSoTData::default();
        }

        match fs::read_to_string(&self.file_path) {
            Ok(content) => {
                match serde_json::from_str::<SSoTData>(&content) {
                    Ok(data) => {
                        info!("📖 SSoT erfolgreich ut {:?} laaden.", self.file_path);
                        data
                    }
                    Err(e) => {
                        error!("❌ JSON-Fehler in SSoT: {}", e);
                        SSoTData::default()
                    }
                }
            }
            Err(e) => {
                error!("❌ Fehlgriff bi't Leesen vun de SSoT: {}", e);
                SSoTData::default()
            }
        }
    }

    /// Schrifft den Tostand op de Platt (Ersatz för save_to_ssot)
    pub fn save_to_ssot(&self, mut data: SSoTData) -> bool {
        data.last_update = Some(Local::now().to_rfc3339());

        match serde_json::to_string_pretty(&data) {
            Ok(content) => {
                if let Err(e) = fs::write(&self.file_path, content) {
                    error!("❌ Kun de SSoT nich schrieven: {}", e);
                    false
                } else {
                    info!("💾 SSoT erfolgreich op Festplatt spiekert.");
                    true
                }
            }
            Err(e) => {
                error!("❌ Serialisierungs-Fehler bi SSoT: {}", e);
                false
            }
        }
    }
}