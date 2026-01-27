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
    pub backend_host: String,
    pub backend_port: u16,
    pub master_ip: String,
    pub heartbeat_hz: f64,
    pub timeout_factor: f64,
    pub domain_ids: HashMap<String, i32>, // Wir nutzen String als Key für Serde-Kompatibilität
}

impl Default for Settings {
    fn default() -> Self {
        let mut domain_ids = HashMap::new();
        domain_ids.insert("hardware".to_string(), 0);
        domain_ids.insert("simulation".to_string(), 1);

        Self {
            current_mode: SystemMode::Hardware,
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
    /// Bootstrapping der Config (Ersatz für SettingsConfigDict)
    pub fn new() -> Result<Self, ConfigError> {
        let s = ConfigLoader::builder()
            // Defaults laden
            .add_source(config::Config::try_from(&Settings::default())?)
            // .env Datei laden
            .add_source(File::with_name(".env").required(false))
            // Environment Variablen (z.B. PASSFORM_BACKEND_PORT)
            .add_source(Environment::with_prefix("PASSFORM").separator("_"))
            .build()?;

        let settings: Settings = s.try_deserialize()?;

        // Validierung (Ersatz für @field_validator)
        if settings.master_ip == "127.0.0.1" {
            warn!("⚠️ MASTER_IP steht auf Localhost. Externe Clients finden das Backend so nicht!");
        }

        info!("✅ Infrastruktur-Config geladen (Modus: {})", settings.current_mode);
        info!("📡 Heartbeat-SSoT: {}Hz (Timeout-Faktor: {})", settings.heartbeat_hz, settings.timeout_factor);

        Ok(Self {
            settings: RwLock::new(settings),
        })
    }

    // --- GETTER & SETTER ---

    pub async fn get_current_mode(&self) -> SystemMode {
        self.settings.read().await.current_mode
    }

    pub async fn get_domain_id(&self) -> i32 {
        let s = self.settings.read().await;
        let mode_key = s.current_mode.to_string();
        *s.domain_ids.get(&mode_key).unwrap_or(&0)
    }

    pub async fn get_heartbeat_period(&self) -> f64 {
        let hz = self.settings.read().await.heartbeat_hz;
        1.0 / hz
    }

    pub async fn set_mode(&self, mode: SystemMode) -> bool {
        let mut s = self.settings.write().await;
        if s.current_mode != mode {
            let old_mode = s.current_mode;
            s.current_mode = mode;
            info!("🔄 Modus-Wechsel: {} -> {} (ROS Domain {})", 
                old_mode, mode, self.get_domain_id_internal(&s));
            return true;
        }
        false
    }

    // Interne Hilfsfunktion für den Zugriff ohne neuen Read-Lock
    fn get_domain_id_internal(&self, s: &Settings) -> i32 {
        let mode_key = s.current_mode.to_string();
        *s.domain_ids.get(&mode_key).unwrap_or(&0)
    }
}