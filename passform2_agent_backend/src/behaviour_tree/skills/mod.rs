use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;
use tracing::{info, warn};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PrimitiveEntry {
    #[serde(rename = "type")]
    pub skill_type: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SkillMetadata {
    #[serde(default)]
    pub name: String,
    
    #[serde(rename = "type")]
    pub skill_type: String,
    
    pub id_short: Option<String>,
    
    pub driver_topic: Option<String>,

    // Für zusammengesetzte Skills (Sequenzen)
    pub primitives: Option<Vec<PrimitiveEntry>>,

    // Für Validierung und UI-Beschreibung (Range, Property etc.)
    pub properties: Option<serde_json::Value>,

    // Für vordefinierte Positionen (wie in move_arm)
    pub predefined_poses: Option<serde_json::Value>,
}

pub struct SkillLibrary {
    pub skills: Vec<SkillMetadata>,
}

impl SkillLibrary {
    pub fn from_workspace() -> Self {
        let mut skills = Vec::new();
        let path = PathBuf::from("../passform2_ws/src/passform_agent_resources/skills");

        if let Ok(abs_path) = fs::canonicalize(&path) {
            info!("🔍 Suche Skills in: {:?}", abs_path);
        }

        if let Ok(entries) = fs::read_dir(&path) {
            for entry in entries.flatten() {
                let file_path = entry.path();
                
                // Wir filtern auf .yaml und ignorieren versteckte Dateien
                if file_path.extension().and_then(|s| s.to_str()) == Some("yaml") {
                    if let Ok(content) = fs::read_to_string(&file_path) {
                        match serde_yaml::from_str::<SkillMetadata>(&content) {
                            Ok(mut skill) => {
                                // NAMENS-LOGIK: Präfixe entfernen und id_short priorisieren
                                let file_stem = file_path.file_stem()
                                    .unwrap_or_default()
                                    .to_string_lossy();

                                // 1. Wenn id_short da ist, nimm das
                                // 2. Sonst nimm den Dateinamen ohne Präfixe
                                if let Some(ref id) = skill.id_short {
                                    skill.name = id.clone();
                                } else {
                                    skill.name = file_stem
                                        .replace("behaviour_tree__", "")
                                        .replace("robot_arm__", "");
                                }

                                info!("✅ Skill geladen: {} (Typ: {}, Datei: {})", 
                                    skill.name, 
                                    skill.skill_type,
                                    file_path.file_name().unwrap().to_string_lossy()
                                );
                                skills.push(skill);
                            }
                            Err(e) => warn!("⚠️ Fehler beim Parsen von {:?}: {}", file_path.file_name().unwrap(), e),
                        }
                    }
                }
            }
        } else {
            warn!("⚠️ Skill-Verzeichnis nicht gefunden: {:?}", path);
        }
        Self { skills }
    }
}