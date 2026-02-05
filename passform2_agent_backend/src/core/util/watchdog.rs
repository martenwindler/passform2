use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};
use tracing::{info, warn};
// Importiere Status aus deinem core::types Modul
use crate::{AppState, SystemRole, Status};

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
            timeout: Duration::from_secs(3), // 3 Sekunden ohne Update = Stale
        }
    }

    pub async fn run(&self) {
        info!("🛡️ System-Watchdog aktiv (Prüfintervall: 500ms)");
        let mut interval = tokio::time::interval(Duration::from_millis(500));

        while self.is_running.load(Ordering::SeqCst) {
            interval.tick().await;

            let role = self.state.infra_config.get_role().await;
            match role {
                SystemRole::Master => self.check_agent_entries().await,
                SystemRole::Client => self.check_master_connection().await,
            }
        }
    }

    /// MASTER-LOGIK: Prüft alle AgentEntry-Objekte im AgentManager
    async fn check_agent_entries(&self) {
        let mut agents = self.state.agent_manager.agents.write().await;
        let now = Instant::now();
        let mut changes_detected = false;

        // DEBUG-LOG: Damit du siehst, dass er prüft
        if !agents.is_empty() {
            info!("🕵️ Watchdog: Prüfe {} Agenten...", agents.len());
        }

        for (id, agent) in agents.iter_mut() {
            let elapsed = now.duration_since(agent.last_seen);

            if elapsed > self.timeout {
                if agent.status != Status::Stale && agent.status != Status::Error {
                    agent.status = Status::Stale;
                    warn!("⚠️ WATCHDOG: Agent '{}' ist jetzt STALE (Letzter Kontakt vor {:?})", id, elapsed);
                    changes_detected = true;
                }
            } else {
                // Optional: Zeige an, dass der Agent gesund ist
                // info!("✅ Agent '{}' ist aktiv (letzter Kontakt vor {:?})", id, elapsed);
            }
        }

        if changes_detected {
            let agents_list: Vec<_> = agents.values().cloned().collect();
            let _ = self.state.socket_manager.emit_event(
                "active_agents", 
                serde_json::json!({ "agents": agents_list })
            ).await;
        }
    }

    async fn check_master_connection(&self) {
        // Logik für den Ranger-Client (Master-Heartbeat prüfen)
    }

    pub fn stop(&self) {
        self.is_running.store(false, Ordering::SeqCst);
    }
}