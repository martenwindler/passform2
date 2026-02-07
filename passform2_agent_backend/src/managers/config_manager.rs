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
    /// Konfiguration (Grid-Größe etc.) - Fallback auf Default bei Fehlen
    #[serde(default = "default_config_val")]
    pub config: Value,
    
    /// Liste der Agenten - Fallback auf leere Liste
    #[serde(default)]
    pub agents: Vec<Value>,
    
    /// Liste der Buchten - Fallback auf leere Liste
    #[serde(default)]
    pub bays: Vec<Value>,
    
    /// Zeitstempel der letzten Änderung
    #[serde(default)]
    pub last_update: Option<String>,
}

fn default_config_val() -> Value {
    json!({"grid": {"width": 10, "height": 10}})
}

impl Default for SSoTData {
    fn default() -> Self {
        Self {
            config: default_config_val(),
            agents: vec![],
            bays: vec![],
            last_update: None,
        }
    }
}

// --- MANAGER ---

pub struct ConfigManager {
    pub initial_path: PathBuf,
    pub session_path: PathBuf,
}

impl ConfigManager {
    pub fn new() -> Self {
        let mut base_path = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
        
        // PFAD-FIX: Wenn wir im Root des Projekts sind, müssen wir in den Backend-Ordner wechseln
        if base_path.join("passform2_agent_backend").exists() {
            base_path.push("passform2_agent_backend");
        }
        
        base_path.push("data");

        let manager = Self {
            initial_path: base_path.join("config__initial__00.json"),
            session_path: base_path.join("config__session__00.json"),
        };
        
        manager.ensure_data_dir();
        manager
    }

    /// Stellt sicher, dass der Datenordner existiert
    fn ensure_data_dir(&self) {
        if let Some(parent) = self.initial_path.parent() {
            if !parent.exists() {
                info!("📁 Erstelle Daten-Verzeichnis: {:?}", parent);
                let _ = fs::create_dir_all(parent);
            }
        }
    }

    /// Lädt die Konfiguration. Priorität: Session > Initial > Default.
    pub fn load_config(&self) -> SSoTData {
        let path_to_load = if self.session_path.exists() {
            info!("🔄 Lade aktive Session: {:?}", self.session_path);
            &self.session_path
        } else {
            info!("🏠 Lade Initial-Stand: {:?}", self.initial_path);
            &self.initial_path
        };

        if !path_to_load.exists() {
            warn!("⚠️ Keine Konfigurationsdatei gefunden unter {:?}", path_to_load);
            return SSoTData::default();
        }

        match fs::read_to_string(path_to_load) {
            Ok(content) => serde_json::from_str::<SSoTData>(&content).unwrap_or_else(|e| {
                error!("❌ JSON-Parse-Fehler in {:?}: {}", path_to_load, e);
                SSoTData::default()
            }),
            Err(e) => {
                error!("❌ Lesefehler bei {:?}: {}", path_to_load, e);
                SSoTData::default()
            }
        }
    }

    /// "Neues Projekt": Löscht die Arbeitskopie (Session)
    pub fn reset_session(&self) -> bool {
        if self.session_path.exists() {
            match fs::remove_file(&self.session_path) {
                Ok(_) => {
                    info!("🗑️ Session-Datei erfolgreich gelöscht.");
                    true
                }
                Err(e) => {
                    error!("❌ Fehler beim Löschen der Session: {}", e);
                    false
                }
            }
        } else {
            true
        }
    }

    /// Schreibt Daten in die config__session__00.json (Arbeitsbereich)
    pub fn save_config(&self, full_data: &Value) -> bool {
        match serde_json::from_value::<SSoTData>(full_data.clone()) {
            Ok(mut data) => {
                data.last_update = Some(Local::now().to_rfc3339());
                self.write_to_file(&self.session_path, &data)
            }
            Err(e) => {
                error!("❌ SSoT-Mapping Fehler (Session): {}", e);
                false
            }
        }
    }

    /// REWRITE-FUNKTION: Schreibt Daten permanent in die config__initial__00.json
    pub fn save_as_initial(&self, full_data: &Value) -> bool {
        match serde_json::from_value::<SSoTData>(full_data.clone()) {
            Ok(mut data) => {
                data.last_update = Some(Local::now().to_rfc3339());
                if self.write_to_file(&self.initial_path, &data) {
                    info!("👑 GOLDEN MASTER AKTUALISIERT: {:?}", self.initial_path);
                    true
                } else {
                    false
                }
            }
            Err(e) => {
                error!("❌ SSoT-Mapping Fehler (Master Rewrite): {}", e);
                false
            }
        }
    }

    /// Private Hilfsfunktion zum sauberen Schreiben auf Festplatte
    fn write_to_file(&self, path: &PathBuf, data: &SSoTData) -> bool {
        match serde_json::to_string_pretty(data) {
            Ok(content) => {
                if let Err(e) = fs::write(path, content) {
                    error!("❌ Fehler beim Schreiben der Datei {:?}: {}", path, e);
                    false
                } else {
                    info!("💾 Datei erfolgreich gespeichert: {:?}", path);
                    true
                }
            }
            Err(e) => {
                error!("❌ Serialisierungs-Fehler: {}", e);
                false
            }
        }
    }
}