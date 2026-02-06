// passform2_agent_backend/src/managers/config_manager.rs

use std::fs;
use std::path::PathBuf;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use tracing::{info, error, warn};
use chrono::Local;

// --- DATENSTRUKTUR FÜR DIE SSOT ---

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SSoTData {
    pub config: Value,
    pub agents: Vec<Value>,
    pub bays: Vec<Value>,
    pub last_update: Option<String>,
}

fn default_config() -> Value {
    serde_json::json!({"grid": {"width": 10, "height": 10}})
}

impl Default for SSoTData {
    fn default() -> Self {
        // Hier definieren wir den "Werkszustand", falls keine Datei existiert
        Self {
            config: json!({"grid": {"width": 10, "height": 10}}),
            agents: vec![], // Wird beim ersten Start durch das System befüllt
            bays: vec![],   // Wird beim ersten Start durch das System befüllt
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
        // "current_dir" ist der Ordner, von dem aus du "cargo run" startest
        let mut path = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
        
        path.push("data"); // Ordnername
        path.push("config__initial__00.json"); 

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

    /// Lädt die SSoT Datei
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

    /// Speichert den aktuellen Zustand (Agenten & Bays) zurück in die JSON
    pub fn save_state(&self, agents: Vec<Value>, bays: Vec<Value>) -> bool {
        let data = SSoTData {
            config: json!({"grid": {"width": 10, "height": 10}}),
            agents,
            bays,
            last_update: Some(Local::now().to_rfc3339()),
        };

        match serde_json::to_string_pretty(&data) {
            Ok(content) => {
                if let Err(e) = fs::write(&self.file_path, content) {
                    error!("❌ Kun de SSoT nich schrieven: {}", e);
                    false
                } else {
                    info!("💾 SSoT (Agents & Bays) erfolgreich spiekert.");
                    true
                }
            }
            Err(e) => {
                error!("❌ Serialisierungs-Fehler: {}", e);
                false
            }
        }
    }

    // Nimmt ein komplettes JSON-Value (vom Frontend), validiert es gegen SSoTData und schreibt es sicher auf die Festplatte.
    pub fn save_config(&self, full_data: &Value) -> bool {
        match serde_json::from_value::<SSoTData>(full_data.clone()) {
            Ok(mut data) => {
                data.last_update = Some(Local::now().to_rfc3339());
                if let Ok(content) = serde_json::to_string_pretty(&data) {
                    if fs::write(&self.file_path, content).is_ok() {
                        info!("💾 SSoT-Datei erfolgreich aktualisiert ({} Agenten).", data.agents.len());
                        return true;
                    }
                }
                false
            }
            Err(e) => {
                error!("❌ SSoT-Mapping fehlgeschlagen (Struktur-Konflikt): {}", e);
                false
            }
        }
    }
}