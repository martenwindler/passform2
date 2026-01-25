pub mod core;
pub mod managers;
pub mod ros;

// Re-exports
pub use crate::core::config::ConfigManager;
pub use crate::managers::AgentManager;
pub use crate::managers::SystemApi;
pub use crate::managers::path_manager::PathManager;

pub struct AppState {
    pub ros_client: std::sync::Arc<crate::ros::ros_client::RosClient>,
    pub agent_manager: std::sync::Arc<crate::managers::AgentManager>,
    pub path_manager: std::sync::Arc<crate::managers::path_manager::PathManager>, 
}