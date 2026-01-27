pub mod core;
pub mod com;
// pub mod ros;      // Temporär deaktiviert
// pub mod managers; // Temporär deaktiviert

use tokio::sync::RwLock;
use std::collections::HashMap;
use serde::{Serialize, Deserialize};
use reqwest::Client;

// Wir definieren die Kern-Strukturen hier in der Lib, 
// damit main.rs und proxy.rs sie beide sehen.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Agent {
    pub agent_id: String,
    pub x: i32,
    pub y: i32,
}

pub struct AppState {
    pub hardware_registry: RwLock<HashMap<String, serde_json::Value>>,
    pub agents: RwLock<Vec<Agent>>,
    pub system_mode: RwLock<String>,
    pub http_client: Client,
}