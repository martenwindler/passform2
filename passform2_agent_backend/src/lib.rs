pub mod behaviour_tree;
pub mod managers;
pub mod ros;

// Hier lag der Hauptfehler: core muss alle Untermodule (util, config, types) deklarieren
pub mod core {
    pub mod types;
    pub mod config;
    pub mod util; // Damit sanitize_id und LogLevel gefunden werden
    pub mod logic; // Falls MatchManager darauf zugreift
}

use tokio::sync::RwLock;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use reqwest::Client;

// --- RE-EXPORTS DER TYPEN (Wichtig für MatchManager & AgentManager) ---
// Wir exportieren Status und SkillType direkt auf die oberste Ebene von core::types
pub use crate::core::types::{Status, ConnectionStatus, SkillType};

// --- RE-EXPORTS DER MANAGER ---
pub use crate::managers::{
    resource_manager::ResourceManager,
    agent_manager::AgentManager,
    agent_manager::AgentEntry,
    config_manager::ConfigManager as DataConfigManager,
    node_manager::NodeManager,
    path_manager::PathManager,
    skill_manager::SkillActionManager, 
    socket_io_manager::SocketIoManager,
    system_api::SystemApi as SystemApiManager, 
    match_manager::MatchManager
};

// Core, ROS
pub use crate::core::config::{SystemRole, SystemMode};
pub use crate::core::config::ConfigManager as InfraConfigManager;
pub use crate::ros::ros_client::RosClient;

// BehaviourTree
pub use crate::behaviour_tree::WorldState;
pub use crate::behaviour_tree::HTNPlanner;

// --- GLOBALER APP-STATE (SSoT) ---

pub struct AppState {
    pub world_state: RwLock<WorldState>,
    pub planner: Arc<HTNPlanner>,
    pub hardware_registry: RwLock<HashMap<String, serde_json::Value>>,
    pub resource_manager: Arc<ResourceManager>,
    pub agent_manager: Arc<AgentManager>,
    pub infra_config: Arc<InfraConfigManager>,
    pub http_client: Client,
    pub socket_manager: Arc<SocketIoManager>,
    pub skill_manager: Arc<SkillActionManager>, 
    pub match_manager: Arc<RwLock<Option<Arc<MatchManager>>>>,
    pub path_manager: Arc<PathManager>,
    pub node_manager: Arc<NodeManager>,
    pub data_config: Arc<DataConfigManager>,
    pub system_api: Arc<crate::managers::system_api::SystemApi>,
    pub ros_client: Arc<RosClient>,
}