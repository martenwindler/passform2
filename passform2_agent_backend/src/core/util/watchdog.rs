use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};
use tracing::{info, warn};
use crate::{AppState, SystemRole};

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

        for (id, agent) in agents.iter_mut() {
            // Prüfung: Wie lange ist der letzte Kontakt her?
            let elapsed = now.duration_since(agent.last_seen);

            if elapsed > self.timeout {
                // Nur reagieren, wenn der Status nicht bereits "stale" oder "offline" ist
                if agent.status != "stale" && agent.status != "offline" {
                    agent.status = "stale".to_string();
                    warn!("⚠️ WATCHDOG: Agent '{}' ist jetzt STALE (Letzter Kontakt: {:?})", id, elapsed);
                    changes_detected = true;
                }
            }
        }

        // Wenn sich Stati geändert haben, senden wir ein Update ans Dashboard
        if changes_detected {
            // Wir nutzen die broadcast-Logik über das Socket-Handle
            let agents_list: Vec<_> = agents.values().cloned().collect();
            let _ = self.state.socket_manager.io.emit(
                "active_agents", 
                serde_json::json!({ "agents": agents_list })
            );
        }
    }

    async fn check_master_connection(&self) {
        // Logik für den Ranger-Client (Master-Heartbeat prüfen)
    }

    pub fn stop(&self) {
        self.is_running.store(false, Ordering::SeqCst);
    }
}