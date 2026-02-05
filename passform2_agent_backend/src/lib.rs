pub mod behaviour_tree;
pub mod managers;
pub mod ros;

pub mod core {
    pub mod types;
    pub mod config;
    pub mod util;
    pub mod logic;
}

use tokio::sync::RwLock;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use reqwest::Client;

// --- RE-EXPORTS DER TYPEN ---
pub use crate::core::types::{Status, ConnectionStatus, SkillType, Bay}; // Bay hinzugefügt
pub use crate::core::types::plant_model::PlantModel;

// --- RE-EXPORTS DER MANAGER ---
pub use crate::managers::{
    resource_manager::ResourceManager,
    agent_manager::AgentManager,
    config_manager::ConfigManager as DataConfigManager,
    node_manager::NodeManager,
    path_manager::PathManager,
    skill_manager::SkillActionManager, 
    socket_io_manager::SocketIoManager,
    system_api::SystemApi, // Vereinheitlicht
    match_manager::MatchManager,
    rfid_manager::RfidManager,
    inventory_manager::InventoryManager,
    basyx_manager::BasyxManager 
};

// Core, ROS
pub use crate::core::config::{SystemRole, SystemMode};
pub use crate::core::config::ConfigManager as InfraConfigManager;
pub use crate::ros::ros_client::RosClient;
pub use crate::ros::RosConverter;

// BehaviourTree
pub use crate::behaviour_tree::WorldState;
pub use crate::behaviour_tree::HTNPlanner;

// --- GLOBALER APP-STATE (SSoT) ---

pub struct AppState {
    pub world_state: RwLock<WorldState>,
    pub system_api: Arc<crate::managers::system_api::SystemApi>,
    pub resource_manager: Arc<ResourceManager>,
    pub ros_client: Arc<RosClient>,
    pub planner: Arc<HTNPlanner>,
    pub hardware_registry: RwLock<HashMap<String, serde_json::Value>>,
    pub agent_manager: Arc<AgentManager>,
    pub infra_config: Arc<InfraConfigManager>,
    pub http_client: Client,
    pub socket_manager: Arc<SocketIoManager>,
    pub skill_manager: Arc<SkillActionManager>, 
    pub match_manager: Arc<RwLock<Option<Arc<MatchManager>>>>,
    pub path_manager: Arc<PathManager>,
    pub node_manager: Arc<NodeManager>,
    pub data_config: Arc<DataConfigManager>,
    pub rfid_manager: Arc<RfidManager>,
    pub inventory_manager: Arc<InventoryManager>,
    // --- NEU: Die fehlenden Bausteine für den Digital Twin ---
    pub plant_model: Arc<RwLock<PlantModel>>,
    pub basyx_manager: Arc<BasyxManager>,
}