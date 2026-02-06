use std::time::Instant;
use serde::{Deserialize, Serialize};
use crate::core::types::Status;

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AgentEntry {
    pub agent_id: Option<String>,
    pub module_type: String,
    pub x: i32,
    pub y: i32,
    pub z: f64,
    pub level: i32, 
    pub orientation: i32,
    pub status: Status,
    pub is_dynamic: bool,
    pub payload: Option<serde_json::Value>,
    
    #[serde(skip, default = "Instant::now")]
    pub last_seen: Instant,
}

impl AgentEntry {
    pub fn new(agent_id: String, module_type: String, x: i32, y: i32, level: i32) -> Self {
        Self {
            agent_id: Some(agent_id),
            module_type,
            x,
            y,
            z: (level * 400) as f64,
            level,
            orientation: 0,
            last_seen: std::time::Instant::now(),
            status: Status::Ok,
            is_dynamic: false, // Fehlte
            payload: None,     // Fehlte
        }
    }
}