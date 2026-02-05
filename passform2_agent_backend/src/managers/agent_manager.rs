use chrono::Local;
use std::collections::{HashMap, VecDeque};
use std::time::Instant;
use std::sync::Arc;
use serde::{Deserialize, Serialize};
use tokio::sync::RwLock;
use tracing::{warn, error};

use crate::core::types::AgentEntry;
use crate::managers::socket_io_manager::SocketIoManager;
use crate::managers::resource_manager::ResourceManager; 
use crate::core::types::Status;

#[derive(Serialize, Clone)]
pub struct LogEntry {
    pub message: String,
    pub level: String,
    pub timestamp: String,
}

// --- MANAGER ---

pub struct AgentManager {
    pub agents: RwLock<HashMap<String, AgentEntry>>,
    pub logs: RwLock<VecDeque<LogEntry>>,
    pub socket_manager: Arc<SocketIoManager>,
    pub resource_manager: Arc<ResourceManager>,
    max_logs: usize,
}

impl AgentManager {
    pub fn new(socket_manager: Arc<SocketIoManager>, resource_manager: Arc<ResourceManager>) -> Self {
        Self {
            agents: RwLock::new(HashMap::new()),
            logs: RwLock::new(VecDeque::with_capacity(50)),
            socket_manager,
            resource_manager,
            max_logs: 50,
        }
    }

    /// Hilfsmethode: Rechnet die Signalstärke in das JSON-Objekt ein (Manager-Logik)
    fn agent_to_json(agent: &AgentEntry, timeout_period: f64) -> serde_json::Value {
        let mut val = serde_json::to_value(agent).unwrap_or(serde_json::json!({}));
        
        let elapsed = agent.last_seen.elapsed().as_secs_f64();
        let signal_strength = if elapsed >= timeout_period {
            0
        } else {
            (100.0 * (1.0 - elapsed / timeout_period)) as i32
        };

        if let Some(obj) = val.as_object_mut() {
            obj.insert("signal_strength".to_string(), serde_json::json!(signal_strength));
        }
        val
    }

    async fn broadcast_update(&self) {
        let timeout = 10.0; // Standard-Timeout für Broadcasts
        let agents_json: Vec<serde_json::Value> = {
            let agents_guard = self.agents.read().await;
            agents_guard.values()
                .map(|a| Self::agent_to_json(a, timeout))
                .collect()
        };
        
        self.socket_manager.emit_event(
            "active_agents",
            serde_json::json!({ "agents": agents_json })
        ).await;
    }

    pub async fn add_agent(&self, id: String, m_type: String, x: i32, y: i32) {
        let m_type_lower = m_type.to_lowercase();
        
        if self.resource_manager.agent_configs.contains_key(&m_type_lower) {
            {
                let mut agents = self.agents.write().await;
                let new_agent = AgentEntry::new(id.clone(), m_type, x, y);
                agents.insert(id.clone(), new_agent);
            }
            self.log_to_system(format!("🚀 Modul {} vom Typ {} erstellt.", id, m_type_lower), "success").await;
            self.broadcast_update().await;
        } else {
            self.log_to_system(format!("⚠️ Fehler: Modul-Typ '{}' ist nicht registriert!", m_type), "error").await;
            error!("Versuch unregistrierten Agent-Typ zu laden: {}", m_type);
        }
    }

    pub async fn sync_from_ros(&self, id: String, m_type: String, x: i32, y: i32, orient: i32, status_code: i32) {
        let m_type_lower = m_type.to_lowercase();
        if !self.resource_manager.agent_configs.contains_key(&m_type_lower) {
            warn!("ROS-Sync für unbekannten Typ: {}", m_type);
        }

        {
            let mut agents = self.agents.write().await;
            let agent = agents.entry(id.clone()).or_insert_with(|| {
                AgentEntry::new(id, m_type, x, y)
            });
            
            agent.x = x;
            agent.y = y;
            agent.orientation = orient;
            agent.last_seen = Instant::now();
            agent.status = Status::from(status_code); 
        }
        self.broadcast_update().await;
    }

    pub async fn execute_mission(&self, ftf_id: String, path: Vec<(i32, i32)>, _start_pos: (i32, i32), _goal_pos: (i32, i32)) {
        self.log_to_system(format!("FTF {} führt Mission aus...", ftf_id), "info").await;

        for (nx, ny) in path {
            {
                let mut agents = self.agents.write().await;
                if let Some(ftf) = agents.get_mut(&ftf_id) {
                    ftf.x = nx;
                    ftf.y = ny;
                    ftf.status = Status::Running;
                    ftf.last_seen = Instant::now();
                }
            }
            self.broadcast_update().await;
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        }
        
        if let Some(ftf) = self.agents.write().await.get_mut(&ftf_id) {
            ftf.status = Status::Ok;
            self.broadcast_update().await;
        }
    }

    pub async fn log_to_system(&self, message: String, level: &str) {
        let mut logs = self.logs.write().await;
        let entry = LogEntry {
            message,
            level: level.to_string(),
            timestamp: Local::now().format("%H:%M:%S").to_string(),
        };
        logs.push_back(entry);
        if logs.len() > self.max_logs {
            logs.pop_front();
        }
    }

    pub async fn get_active_agents_json(&self, timeout_period: f64) -> serde_json::Value {
        let agents = self.agents.read().await;
        let mut map = serde_json::Map::new();
        
        for a in agents.values() {
            let key = format!("{},{}", a.x, a.y);
            let val = Self::agent_to_json(a, timeout_period);
            map.insert(key, val);
        }
        serde_json::Value::Object(map)
    }
}