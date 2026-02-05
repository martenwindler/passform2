use tracing::info;
use std::time::Instant;
// In sysinfo 0.30+ brauchen wir die Ext-Traits nicht mehr explizit importieren
use sysinfo::System; 
use serde::Serialize;

#[derive(Serialize)]
pub struct SystemStats {
    pub uptime: u64,
    pub memory_used_mb: u64,
    pub cpu_usage_percent: f32,
    pub health_status: String,
}

pub struct SystemApi {
    start_time: Instant,
}

impl SystemApi {
    pub fn new() -> Self {
        info!("🖥️ System-API initialisiert.");
        Self {
            start_time: Instant::now(),
        }
    }

    pub async fn get_stats(&self) -> SystemStats {
        // Wir erstellen ein neues System-Objekt
        let mut sys = System::new_all();
        
        // Wichtig: In 0.30+ heißt es einfach refresh_all() ohne Trait-Import
        sys.refresh_all();
        
        // Speicher in MB (sys.used_memory() gibt Bytes zurück)
        let used_memory = sys.used_memory() / 1024 / 1024;
        
        // CPU Usage Snapshot
        let cpu_usage = sys.global_cpu_usage();

        SystemStats {
            uptime: self.start_time.elapsed().as_secs(),
            memory_used_mb: used_memory,
            cpu_usage_percent: cpu_usage,
            health_status: "Healthy".to_string(),
        }
    }

    pub async fn get_uptime(&self) -> u64 {
        self.start_time.elapsed().as_secs()
    }

    pub async fn check_health(&self) -> bool {
        true
    }
}

impl Default for SystemApi {
    fn default() -> Self {
        Self::new()
    }
}