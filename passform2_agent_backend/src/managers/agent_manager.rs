use chrono::Local;
use std::collections::{HashMap, VecDeque};
use std::time::Instant;
use std::sync::Arc;
use serde::{Deserialize, Serialize};
use tokio::sync::RwLock;
use tracing::{info, warn, error};

use crate::managers::socket_io_manager::SocketIoManager;
use crate::managers::resource_manager::ResourceManager; 

// --- DOMAIN MODELS ---

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AgentEntry {
    pub agent_id: String,
    pub module_type: String,
    pub x: i32,
    pub y: i32,
    pub orientation: i32,
    pub status: String,
    pub is_dynamic: bool,
    pub payload: Option<serde_json::Value>,
    
    #[serde(skip, default = "Instant::now")]
    pub last_seen: Instant,
}

impl AgentEntry {
    pub fn new(agent_id: String, module_type: String, x: i32, y: i32) -> Self {
        Self {
            agent_id: agent_id.clone(),
            is_dynamic: module_type.to_lowercase() == "ftf",
            module_type,
            x,
            y,
            orientation: 0,
            status: "active".to_string(),
            payload: None,
            last_seen: Instant::now(),
        }
    }

    pub fn get_signal_strength(&self, timeout_period: f64) -> i32 {
        let elapsed = self.last_seen.elapsed().as_secs_f64();
        if elapsed >= timeout_period {
            0
        } else {
            (100.0 * (1.0 - elapsed / timeout_period)) as i32
        }
    }
}

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
    /// Erstellt einen neuen AgentManager mit Socket-Anbindung und ResourceManager
    pub fn new(socket_manager: Arc<SocketIoManager>, resource_manager: Arc<ResourceManager>) -> Self {
        Self {
            agents: RwLock::new(HashMap::new()),
            logs: RwLock::new(VecDeque::with_capacity(50)),
            socket_manager,
            resource_manager, // NEU
            max_logs: 50,
        }
    }

    /// Hilfsfunktion: Sendet den aktuellen Stand aller Agenten an das Frontend
    async fn broadcast_update(&self) {
        let agents_list: Vec<AgentEntry> = {
            let agents_guard = self.agents.read().await;
            agents_guard.values().cloned().collect()
        };
        
        self.socket_manager.emit_event(
            "active_agents",
            serde_json::json!({ "agents": agents_list })
        ).await;
    }

    pub async fn add_agent(&self, id: String, m_type: String, x: i32, y: i32) {
        // VALIDIERUNG: Prüfe im ResourceManager, ob dieser Typ bekannt ist
        let m_type_lower = m_type.to_lowercase();
        
        if self.resource_manager.agent_configs.contains_key(&m_type_lower) {
            {
                let mut agents = self.agents.write().await;
                // Hier könnten wir später entscheiden, ob is_dynamic aus der Config kommt
                let new_agent = AgentEntry::new(id.clone(), m_type, x, y);
                agents.insert(id.clone(), new_agent);
            }
            self.log_to_system(format!("🚀 Modul {} vom Typ {} erstellt.", id, m_type_lower), "success").await;
            self.broadcast_update().await;
        } else {
            // Fehlerlog, wenn der Typ nicht in passform_agent_resources/config gefunden wurde
            self.log_to_system(format!("⚠️ Fehler: Modul-Typ '{}' ist nicht registriert!", m_type), "error").await;
            error!("Versuch unregistrierten Agent-Typ zu laden: {}", m_type);
        }
    }

    // sync_from_ros sollte ebenfalls validieren
    pub async fn sync_from_ros(&self, id: String, m_type: String, x: i32, y: i32, orient: i32) {
        let m_type_lower = m_type.to_lowercase();
        if !self.resource_manager.agent_configs.contains_key(&m_type_lower) {
            warn!("ROS-Sync für unbekannten Typ: {}", m_type);
            // Wir lassen es trotzdem zu, loggen aber eine Warnung
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
            agent.status = "active".to_string();
        }
        self.broadcast_update().await;
    }

    pub async fn execute_mission(&self, ftf_id: String, path: Vec<(i32, i32)>, start_pos: (i32, i32), goal_pos: (i32, i32)) {
        self.log_to_system(format!("FTF {} führt Mission aus...", ftf_id), "info").await;

        for (nx, ny) in path {
            {
                let mut agents = self.agents.write().await;
                
                if let Some(ftf) = agents.get_mut(&ftf_id) {
                    ftf.x = nx;
                    ftf.y = ny;
                    ftf.status = "moving".to_string();
                    ftf.last_seen = Instant::now();
                }

                if (nx, ny) == start_pos {
                    let maybe_payload_id = agents.iter()
                        .find(|(id, a)| !a.is_dynamic && (a.x, a.y) == start_pos && *id != &ftf_id)
                        .map(|(id, _)| id.clone());

                    if let Some(tid) = maybe_payload_id {
                        if let Some(target_agent) = agents.remove(&tid) {
                            if let Some(ftf) = agents.get_mut(&ftf_id) {
                                if ftf.payload.is_none() {
                                    ftf.payload = Some(serde_json::to_value(target_agent).unwrap());
                                    self.log_to_system(format!("📦 Modul {} aufgenommen.", tid), "success").await;
                                }
                            }
                        }
                    }
                }

                if (nx, ny) == goal_pos {
                    let payload_to_drop = agents.get_mut(&ftf_id)
                        .and_then(|ftf| ftf.payload.take());

                    if let Some(p_val) = payload_to_drop {
                        if let Ok(p) = serde_json::from_value::<AgentEntry>(p_val) {
                            let new_p = AgentEntry::new(p.agent_id.clone(), p.module_type, nx, ny);
                            agents.insert(p.agent_id, new_p);
                            self.log_to_system("✅ Modul abgesetzt.".to_string(), "success").await;
                        }
                    }
                }
            }
            self.broadcast_update().await;
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
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
            let mut val = serde_json::to_value(a).unwrap();
            val.as_object_mut().unwrap().insert(
                "signal_strength".to_string(), 
                serde_json::json!(a.get_signal_strength(timeout_period))
            );
            map.insert(key, val);
        }
        serde_json::Value::Object(map)
    }
}