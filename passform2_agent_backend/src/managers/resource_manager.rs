// src/managers/resource_manager.rs

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::fs;
use tracing::{info, warn};

pub struct ResourceManager {
    pub base_path: PathBuf,
    pub skills_path: PathBuf,
    pub urdf_path: PathBuf,
    pub meshes_path: PathBuf, // NEU
    pub config_path: PathBuf,
    pub msg_path: PathBuf,
    pub srv_path: PathBuf,
    pub action_path: PathBuf,
    
    // Registrierte Ressourcen-Definitionen
    pub agent_configs: HashMap<String, serde_json::Value>,
    
    // Listen der gefundenen ROS-Interfaces
    pub available_msgs: Vec<String>,
    pub available_srvs: Vec<String>,
    pub available_actions: Vec<String>,
}

impl ResourceManager {
    pub fn new(base_path: PathBuf) -> Self {
        let skills = base_path.join("skills");
        let urdf = base_path.join("urdf");
        let meshes = base_path.join("meshes"); // NEU
        let config = base_path.join("config");
        let msg = base_path.join("msg");
        let srv = base_path.join("srv");
        let action = base_path.join("action");

        let mut manager = Self {
            base_path,
            skills_path: skills,
            urdf_path: urdf,
            meshes_path: meshes, // NEU
            config_path: config,
            msg_path: msg,
            srv_path: srv,
            action_path: action,
            agent_configs: HashMap::new(),
            available_msgs: Vec::new(),
            available_srvs: Vec::new(),
            available_actions: Vec::new(),
        };

        manager.initialize();
        manager
    }

    fn initialize(&mut self) {
        info!("📂 ResourceManager: Initialisiere Ressourcen aus {:?}", self.base_path);
        
        // 1. Validiere alle Pfade
        self.check_dir(&self.skills_path, "Skills");
        self.check_dir(&self.urdf_path, "URDFs");
        self.check_dir(&self.meshes_path, "Meshes (3D-Modelle)"); // NEU
        self.check_dir(&self.config_path, "Agent-Configs");
        self.check_dir(&self.msg_path, "Messages (.msg)");
        self.check_dir(&self.srv_path, "Services (.srv)");
        self.check_dir(&self.action_path, "Actions (.action)");

        // 2. Scanne ROS 2 Interface-Dateien
        self.available_msgs = self.scan_interface_dir(&self.msg_path, "msg");
        self.available_srvs = self.scan_interface_dir(&self.srv_path, "srv");
        self.available_actions = self.scan_interface_dir(&self.action_path, "action");

        // 3. Lade Agenten-Konfigurationen
        self.load_agent_configs();
        
        info!("✅ Resource Scan abgeschlossen: {} Configs, {} Msgs, {} Srvs, {} Actions.", 
            self.agent_configs.len(),
            self.available_msgs.len(),
            self.available_srvs.len(),
            self.available_actions.len()
        );
    }

    fn check_dir(&self, path: &Path, label: &str) {
        if path.exists() && path.is_dir() {
            info!("✅ {} Pfad gefunden: {:?}", label, path);
        } else {
            warn!("⚠️ {} Pfad nicht gefunden oder kein Verzeichnis: {:?}", label, path);
        }
    }

    /// Hilfsfunktion zum Scannen von ROS 2 Interface-Dateien
    fn scan_interface_dir(&self, path: &Path, extension: &str) -> Vec<String> {
        let mut found_interfaces = Vec::new();
        if let Ok(entries) = fs::read_dir(path) {
            for entry in entries.flatten() {
                let p = entry.path();
                if p.extension().and_then(|s| s.to_str()) == Some(extension) {
                    if let Some(stem) = p.file_stem().and_then(|s| s.to_str()) {
                        info!("📡 ROS Schnittstelle gefunden: {}.{}", stem, extension);
                        found_interfaces.push(stem.to_string());
                    }
                }
            }
        }
        found_interfaces
    }

    fn load_agent_configs(&mut self) {
        if let Ok(entries) = fs::read_dir(&self.config_path) {
            for entry in entries.flatten() {
                let path = entry.path();
                let is_valid_ext = path.extension().and_then(|s| s.to_str())
                    .map(|ext| ext == "yaml" || ext == "json")
                    .unwrap_or(false);

                if is_valid_ext {
                    if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                        info!("📑 Lade Agenten-Spezifikation: {}", stem);
                        self.agent_configs.insert(
                            stem.to_string(), 
                            serde_json::json!({ "path": path.to_string_lossy() })
                        );
                    }
                }
            }
        }
    }
}