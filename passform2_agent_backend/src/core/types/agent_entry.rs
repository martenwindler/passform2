use std::time::Instant;
use serde::{Deserialize, Serialize};
use crate::core::types::Status;

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AgentEntry {
    pub agent_id: String,
    pub module_type: String,
    pub x: i32,
    pub y: i32,
    pub orientation: i32,
    pub status: Status,
    pub is_dynamic: bool,
    pub payload: Option<serde_json::Value>,
    
    #[serde(skip, default = "Instant::now")]
    pub last_seen: Instant,
}

impl AgentEntry {
    pub fn new(agent_id: String, module_type: String, x: i32, y: i32) -> Self {
        let is_dynamic = matches!(module_type.to_lowercase().as_str(), "ftf" | "ranger");
        Self {
            agent_id,
            is_dynamic,
            module_type,
            x,
            y,
            orientation: 0,
            status: Status::Ok,
            payload: None,
            last_seen: Instant::now(),
        }
    }
}