use std::collections::HashMap;
use std::process::Stdio;
use std::sync::Arc;
use tokio::process::{Child, Command};
use tokio::sync::RwLock;
use tracing::{info, error, warn};
use std::path::PathBuf;
use serde::Deserialize;

// --- API SCHEMAS ---
#[derive(Deserialize)]
pub struct StartAgentRequest {
    pub agent_id: String,
    pub x: i32,
    pub y: i32,
    pub module_type: String,
}

// --- MANAGER ---
pub struct NodeManager {
    // Wi spiekert de Child-Prozesse mit de Agent-ID as Key
    pub processes: Arc<RwLock<HashMap<String, Child>>>,
}

impl NodeManager {
    pub fn new() -> Self {
        info!("✅ NodeManager Orchestrator op de Brücke bereit.");
        Self {
            processes: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Ermittelt de Projekt-Wurzel (analog to _get_project_root)
    fn get_project_root(&self) -> PathBuf {
        std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."))
    }

    // --- SYSTEM NODES ---

    pub async fn start_system_nodes(&self, domain_id: u16, hz: f64, factor: f64) {
        let root = self.get_project_root();
        let script_path = root.join("passform2_ws/src/passform_agent_planning/passform_agent_planning/monitor_node.py");

        if !script_path.exists() {
            error!("❌ Monitor-Script nich funnen an {:?}", script_path);
            return;
        }

        self.spawn_process(
            "system_monitor".to_string(),
            "python3".to_string(),
            vec![
                script_path.to_string_lossy().to_string(),
                "--ros-args".into(),
                "-p".into(), format!("heartbeat_hz:={}", hz),
                "-p".into(), format!("timeout_factor:={}", factor),
            ],
            domain_id,
        ).await;
    }

    // --- PROCESS SPAWNING ---

    async fn spawn_process(&self, id: String, cmd_base: String, args: Vec<String>, domain_id: u16) {
        let mut processes = self.processes.write().await;
        
        // Dubbel-Check: Löppt dat al?
        if processes.contains_key(&id) { return; }

        let mut command = Command::new(cmd_base);
        command
            .args(args)
            .env("ROS_DOMAIN_ID", domain_id.to_string())
            .stdout(Stdio::null())
            .stderr(Stdio::inherit()) // Fehler wollen wi sehn!
            .kill_on_drop(true); // Sekerheit: Wenn de Manager stirbt, starvt de Nodes ok

        // Linux/Unix spezifisch: Prozess-Gruppe setten (Setsid)
        #[cfg(unix)]
        unsafe {
            command.pre_exec(|| {
                libc::setsid();
                Ok(())
            });
        }

        match command.spawn() {
            Ok(child) => {
                processes.insert(id.clone(), child);
                info!("🖥️ Prozess [{}] gestartet (ROS Domain: {})", id, domain_id);
            },
            Err(e) => error!("❌ Kun den Prozess [{}] nich starten: {}", id, e),
        }
    }

    pub async fn start_node(&self, id: String, x: i32, y: i32, m_type: String, domain_id: u16, hz: f64) {
        let root = self.get_project_root();
        let script_path = root.join("passform2_ws/src/passform_agent_planning/passform_agent_planning/agent_node.py");

        let args = vec![
            script_path.to_string_lossy().to_string(),
            "--ros-args".into(),
            "-p".into(), format!("agent_id:={}", id),
            "-p".into(), format!("module_type:={}", m_type),
            "-p".into(), format!("position:=[{},{}]", x, y),
            "-p".into(), format!("heartbeat_hz:={}", hz),
        ];

        self.spawn_process(id, "python3".to_string(), args, domain_id).await;
    }

    // --- TERMINATION ---

    pub async fn kill_node(&self, id: &str) {
        let mut processes = self.processes.write().await;
        if let Some(mut child) = processes.remove(id) {
            match child.kill().await {
                Ok(_) => info!("🛑 Node {} sauber beendet.", id),
                Err(e) => warn!("⚠️ Fehler bi't Beenden vun {}: {}", id, e),
            }
        }
    }

    pub async fn kill_all_nodes(&self) {
        let mut processes = self.processes.write().await;
        info!("🛑 Herunterfahren aller {} ROS-Nodes...", processes.len());
        for (id, mut child) in processes.drain() {
            let _ = child.kill().await;
            info!("🛑 Node {} gekillt.", id);
        }
    }
}