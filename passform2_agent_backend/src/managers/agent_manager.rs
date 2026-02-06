use chrono::Local;
use std::collections::{HashMap, VecDeque};
use std::time::Instant;
use std::sync::Arc;
use serde::{Deserialize, Serialize};
use tokio::sync::RwLock;
use tracing::{warn, error, info};

use crate::core::types::AgentEntry;
use crate::core::types::plant_model::PlantModel; 
use crate::managers::socket_io_manager::SocketIoManager;
use crate::managers::resource_manager::ResourceManager; 
use crate::core::types::Status;

#[derive(Serialize, Clone)]
pub struct LogEntry {
    pub message: String,
    pub level: String,
    pub timestamp: String,
}

pub struct AgentManager {
    pub agents: RwLock<HashMap<String, AgentEntry>>,
    pub plant: Arc<RwLock<PlantModel>>, 
    pub logs: RwLock<VecDeque<LogEntry>>,
    pub socket_manager: Arc<SocketIoManager>,
    pub resource_manager: Arc<ResourceManager>,
    max_logs: usize,
}

impl AgentManager {
    pub fn new(
        socket_manager: Arc<SocketIoManager>, 
        resource_manager: Arc<ResourceManager>,
        plant: Arc<RwLock<PlantModel>> 
    ) -> Self {
        Self {
            agents: RwLock::new(HashMap::new()),
            plant,
            logs: RwLock::new(VecDeque::with_capacity(50)),
            socket_manager,
            resource_manager,
            max_logs: 50,
        }
    }

    /// Hilfsmethode: Rechnet die Signalstärke in das JSON-Objekt ein
    fn agent_to_json(agent: &AgentEntry, timeout_period: f64) -> serde_json::Value {
        let mut val = serde_json::to_value(agent).unwrap_or(serde_json::json!({}));
        let elapsed = agent.last_seen.elapsed().as_secs_f64();
        let signal_strength = if elapsed >= timeout_period { 0 } else {
            (100.0 * (1.0 - elapsed / timeout_period)) as i32
        };

        if let Some(obj) = val.as_object_mut() {
            obj.insert("signal_strength".to_string(), serde_json::json!(signal_strength));
        }
        val
    }

    /// Vergleicht Agenten-Position mit Bay-Koordinaten (Spatial Mapping)
    async fn update_spatial_mapping(&self, agent_id: &str, x: f64, y: f64) {
        let mut plant_guard = self.plant.write().await;
        let threshold = 0.5; // 50cm Toleranzbereich

        for bay in plant_guard.bays.iter_mut() {
            let dx = x - bay.origin[0];
            let dy = y - bay.origin[1];
            let distance = (dx * dx + dy * dy).sqrt();

            if distance < threshold {
                if !bay.occupation || bay.module_uuid != agent_id {
                    info!("📍 Agent {} hat Bay {} belegt", agent_id, bay.name);
                    bay.occupation = true;
                    bay.module_uuid = agent_id.to_string();
                }
            } else if bay.module_uuid == agent_id {
                info!("💨 Agent {} hat Bay {} verlassen", agent_id, bay.name);
                bay.occupation = false;
                bay.module_uuid = String::new();
            }
        }
    }

    async fn broadcast_update(&self) {
        let timeout = 10.0;
        let agents_json = self.get_active_agents_json(timeout).await;
        
        self.socket_manager.emit_event(
            "active_agents",
            serde_json::json!({ "agents": agents_json })
        ).await;
    }

    pub async fn add_agent(&self, id: String, m_type: String, x: i32, y: i32, lvl: i32) {
        let m_type_lower = m_type.to_lowercase();
        if self.resource_manager.agent_configs.contains_key(&m_type_lower) {
            {
                let mut agents = self.agents.write().await;
                // KORREKTUR: AgentEntry::new mit 5 Parametern aufrufen
                let new_agent = AgentEntry::new(id.clone(), m_type, x, y, lvl);
                agents.insert(id.clone(), new_agent);
            }
            self.update_spatial_mapping(&id, x as f64, y as f64).await;
            self.log_to_system(format!("🚀 Modul {} vom Typ {} erstellt.", id, m_type_lower), "success").await;
            self.broadcast_update().await;
        } else {
            error!("Versuch unregistrierten Agent-Typ zu laden: {}", m_type);
        }
    }

    pub async fn sync_from_ros(&self, id: String, m_type: String, x: i32, y: i32, orient: i32, lvl: i32) {
        {
            let mut agents = self.agents.write().await;
            let agent = agents.entry(id.clone()).or_insert_with(|| {
                // KORREKTUR: AgentEntry::new mit 5 Parametern aufrufen
                AgentEntry::new(id.clone(), m_type, x, y, lvl)
            });
            
            agent.x = x;
            agent.y = y;
            agent.level = lvl; 
            agent.z = (lvl * 400) as f64;
            agent.orientation = orient;
            agent.last_seen = Instant::now();
        }
        
        self.update_spatial_mapping(&id, x as f64, y as f64).await;
        self.broadcast_update().await;
    }

    /// Zentrale Methode für den JSON-Export (wichtig für Elm-Dictionary Mapping)
    pub async fn get_active_agents_json(&self, timeout_period: f64) -> serde_json::Value {
        let agents = self.agents.read().await;
        let mut map = serde_json::Map::new();
        for a in agents.values() {
            // FALSCH: format!("{},{}", a.x, a.y) 
            // RICHTIG:
            let key = format!("{},{},{}", a.x, a.y, a.level); 
            map.insert(key, Self::agent_to_json(a, timeout_period));
        }
        serde_json::Value::Object(map)
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
            self.update_spatial_mapping(&ftf_id, nx as f64, ny as f64).await;
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
}