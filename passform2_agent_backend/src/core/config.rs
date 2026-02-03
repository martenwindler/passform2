use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use tokio::sync::RwLock;
use tracing::{info, warn};
use config::{Config as ConfigLoader, ConfigError, File, Environment};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SystemMode {
    Hardware,
    Simulation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SystemRole {
    Master,
    Client,
}

impl std::fmt::Display for SystemMode {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            SystemMode::Hardware => write!(f, "hardware"),
            SystemMode::Simulation => write!(f, "simulation"),
        }
    }
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Settings {
    pub current_mode: SystemMode,
    pub role: SystemRole, // NEU: Rolle im System
    pub agent_uuid: String, // NEU: Eigene ID für Clients
    pub backend_host: String,
    pub backend_port: u16,
    pub master_ip: String,
    pub heartbeat_hz: f64,
    pub timeout_factor: f64,
    pub domain_ids: HashMap<String, i32>,
}

impl Default for Settings {
    fn default() -> Self {
        let mut domain_ids = HashMap::new();
        domain_ids.insert("hardware".to_string(), 0);
        domain_ids.insert("simulation".to_string(), 1);

        Self {
            current_mode: SystemMode::Hardware,
            role: SystemRole::Client, // Standardmäßig ein Client/Slave
            agent_uuid: "unnamed_agent".to_string(),
            backend_host: "0.0.0.0".to_string(),
            backend_port: 8000,
            master_ip: "127.0.0.1".to_string(),
            heartbeat_hz: 1.0,
            timeout_factor: 3.5,
            domain_ids,
        }
    }
}

pub struct ConfigManager {
    pub settings: RwLock<Settings>,
}

impl ConfigManager {
    pub fn new() -> Result<Self, ConfigError> {
        let s = ConfigLoader::builder()
            .add_source(config::Config::try_from(&Settings::default())?)
            .add_source(File::with_name(".env").required(false))
            // Ermöglicht PASSFORM_ROLE=master oder PASSFORM_AGENT_UUID=ranger_01
            .add_source(Environment::with_prefix("PASSFORM").separator("_"))
            .build()?;

        let settings: Settings = s.try_deserialize()?;

        if settings.role == SystemRole::Master && settings.master_ip == "127.0.0.1" {
            warn!("⚠️ MASTER konfiguriert, aber MASTER_IP ist Localhost. Externe Clients finden dich evtl. nicht.");
        }

        info!("✅ Config geladen | Rolle: {:?} | Modus: {}", settings.role, settings.current_mode);

        Ok(Self {
            settings: RwLock::new(settings),
        })
    }

    pub async fn get_role(&self) -> SystemRole {
        self.settings.read().await.role
    }

    pub async fn get_current_mode(&self) -> SystemMode {
        self.settings.read().await.current_mode
    }

    pub async fn get_domain_id(&self) -> i32 {
        let s = self.settings.read().await;
        let mode_key = s.current_mode.to_string();
        *s.domain_ids.get(&mode_key).unwrap_or(&0)
    }

    pub async fn set_mode(&self, mode: SystemMode) -> bool {
        let mut s = self.settings.write().await;
        if s.current_mode != mode {
            s.current_mode = mode;
            info!("🔄 Modus-Wechsel -> {} (Domain {})", mode, self.get_domain_id_internal(&s));
            return true;
        }
        false
    }

    fn get_domain_id_internal(&self, s: &Settings) -> i32 {
        let mode_key = s.current_mode.to_string();
        *s.domain_ids.get(&mode_key).unwrap_or(&0)
    }
}