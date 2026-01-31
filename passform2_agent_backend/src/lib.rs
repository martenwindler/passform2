pub mod core;
pub mod com;
pub mod managers;
pub mod ros;

use tokio::sync::RwLock;
use std::collections::HashMap;
use std::sync::Arc;
use reqwest::Client;

// --- RE-EXPORTS DER MANAGER ---
// Wir holen die Structs aus ihren jeweiligen Untermodulen in 'managers'
pub use crate::managers::{
    agent_manager::AgentManager,
    agent_manager::AgentEntry,
    config_manager::ConfigManager as DataConfigManager,
    node_manager::NodeManager,
    path_manager::PathManager,
    skill_manager::SkillManager,
    socket_io_manager::SocketIoManager, // Unterstriche korrigiert
    system_api::SystemApi as SystemApiManager, // Name gemappt auf system_api.rs
};

// Auch für Core-Elemente und ROS
pub use crate::core::config::ConfigManager as InfraConfigManager;
pub use crate::ros::ros_client::RosClient;

// --- GLOBALER APP-STATE (SSoT) ---

pub struct AppState {
    /// Dynamische Rohdaten von den Raspberry Pis (Socket.io Registry)
    pub hardware_registry: RwLock<HashMap<String, serde_json::Value>>,
    
    /// Die "High-Level" Agentenliste, verwaltet durch den AgentManager
    pub agent_manager: Arc<AgentManager>,
    
    /// System-Konfiguration (Infrastruktur-Ebene: IP, Ports)
    pub infra_config: Arc<InfraConfigManager>,

    /// HTTP Client für BaSyx Kommunikation
    pub http_client: Client,
    
    /// Manager-Instanzen für die verschiedenen Domänen
    pub socket_manager: Arc<SocketIoManager>,
    pub skill_manager: Arc<SkillManager>,
    pub path_manager: Arc<PathManager>,
    pub node_manager: Arc<NodeManager>,
    pub data_config: Arc<DataConfigManager>, // SSoT (Grid, Agents)
    
    /// ROS 2 Kommunikation (Bridge)
    pub ros_client: Arc<RosClient>,
}