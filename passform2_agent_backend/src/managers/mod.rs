// src/managers/mod.rs
pub mod agent_manager;
pub mod config_manager;
pub mod match_manager;
pub mod node_manager;
pub mod path_manager;
pub mod resource_manager;
pub mod skill_manager;
pub mod socket_io_manager;
pub mod system_api;

// Re-exports für einfacheren Zugriff
pub use agent_manager::AgentManager;
pub use path_manager::PathManager;
pub use system_api::SystemApi;