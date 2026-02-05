use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};
use tracing::{info, warn};

// Importe aus deinem Projekt-Root/Backend
use crate::{AppState, SystemRole};

use crate::core::types::Status;

pub struct Watchdog {
    state: Arc<AppState>,
    is_running: Arc<AtomicBool>,
    timeout: Duration,
}

impl Watchdog {
    pub fn new(state: Arc<AppState>) -> Self {
        Self {
            state,
            is_running: Arc::new(AtomicBool::new(true)),
            timeout: Duration::from_secs(3), // Globales Timeout für Stale-Erkennung
        }
    }

    pub async fn run(&self) {
        info!("🛡️ System-Watchdog aktiv (Prüfintervall: 500ms)");
        let mut interval = tokio::time::interval(Duration::from_millis(500));

        while self.is_running.load(Ordering::SeqCst) {
            interval.tick().await;

            let role = self.state.infra_config.get_role().await;
            match role {
                SystemRole::Master => {
                    // Prüfe sowohl bewegliche Agenten als auch stationäre Bays
                    self.check_agent_entries().await;
                    self.check_bay_entries().await;
                }
                SystemRole::Client => {
                    self.check_master_connection().await;
                }
            }
        }
    }

    /// MASTER-LOGIK: Überwacht Agenten (FTFs, Ranger etc.)
    async fn check_agent_entries(&self) {
        let mut agents = self.state.agent_manager.agents.write().await;
        let now = Instant::now();
        let mut changes_detected = false;

        for (id, agent) in agents.iter_mut() {
            let elapsed = now.duration_since(agent.last_seen);

            if elapsed > self.timeout {
                if agent.status != Status::Stale && agent.status != Status::Error {
                    agent.status = Status::Stale;
                    warn!("⚠️ WATCHDOG: Agent '{}' STALE (Kein Kontakt seit {:?})", id, elapsed);
                    changes_detected = true;
                }
            }
        }

        if changes_detected {
            self.broadcast_agents(&agents.values().cloned().collect()).await;
        }
    }

    /// MASTER-LOGIK: Überwacht die RFID-Buchten (Bays)
    /// Nutzt das gestrige Muster: Logik hier, Bay ist nur Datenstruktur.
    async fn check_bay_entries(&self) {
        let mut bays = self.state.rfid_manager.bays.write().await;
        let now = Instant::now();
        let mut changes_detected = false;

        for (id, bay) in bays.iter_mut() {
            // Nur Bays prüfen, die belegt sind (occupation = true)
            if bay.occupation {
                let elapsed = now.duration_since(bay.last_update);

                if elapsed > self.timeout {
                    // Wenn die Bay belegt ist, aber kein Update mehr kommt -> Timeout/Error
                    if !bay.has_timeout {
                        bay.has_timeout = true;
                        bay.status = Status::Error; 
                        warn!("🚨 WATCHDOG: Bay '{}' TIMEOUT! Verbindung zu Modul {} verloren.", id, bay.module_uuid);
                        changes_detected = true;
                    }
                }
            }
        }

        if changes_detected {
            self.broadcast_bays(&bays.values().cloned().collect()).await;
        }
    }

    // --- Helper für Socket-Broadcasting ---

    async fn broadcast_agents(&self, list: &Vec<crate::core::types::AgentEntry>) {
        let _ = self.state.socket_manager.emit_event(
            "active_agents", 
            serde_json::json!({ "agents": list })
        ).await;
    }

    async fn broadcast_bays(&self, list: &Vec<crate::core::types::bay_aas::Bay>) {
        let _ = self.state.socket_manager.emit_event(
            "bay_update", 
            serde_json::json!({ "bays": list })
        ).await;
    }

    async fn check_master_connection(&self) {
        // Implementierung für Client-Rollen (Heartbeat vom Master prüfen)
    }

    pub fn stop(&self) {
        self.is_running.store(false, Ordering::SeqCst);
    }
}