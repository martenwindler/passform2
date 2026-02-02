#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use axum::{extract::State as AxumState, routing::{get, any, post}, Json, Router};
use socketioxide::{extract::{Data, SocketRef}, SocketIo};
use std::sync::Arc;
use std::net::SocketAddr;
use tracing::{info, error, warn};
use tower_http::{services::{ServeDir, ServeFile}, cors::{Any, CorsLayer}};
use reqwest::Client as HttpClient;
use tokio::sync::RwLock;
use std::collections::HashMap;

// Library-Importe aus dem Backend-Crate
use passform_agent_backend::{
    AppState, SocketIoManager, SkillManager, PathManager, 
    NodeManager, AgentManager, InfraConfigManager, DataConfigManager
};

use passform_agent_backend::ros::ros_client::RosClient;
use passform_agent_backend::managers::path_manager::api_plan_path;
use passform_agent_backend::managers::resource_manager::ResourceManager;
use std::path::PathBuf;

use passform_agent_backend::behaviour_tree::skills::SkillLibrary;
use passform_agent_backend::behaviour_tree::skill_node::SkillNode;
use passform_agent_backend::behaviour_tree::{BehaviorNode, NodeStatus, build_tree}; // build_tree und NodeStatus hinzugefügt

mod com;
use crate::com::proxy::aas_proxy_handler;

// --- SOCKET.IO HANDLER ---

fn setup_socket_handlers(io: &SocketIo, state: Arc<AppState>) {
    io.ns("/", move |socket: SocketRef| {
        let s = state.clone();
        info!("🔗 Socket verbunden: {} | Transport: {:?}", socket.id, socket.transport_type());

        // Initial Sync
        let sync_state = s.clone();
        let sync_socket = socket.clone();
        tokio::spawn(async move {
            let mode = sync_state.infra_config.settings.read().await.current_mode.clone();
            let _ = sync_socket.emit("mode", mode);

            let agents_guard = sync_state.agent_manager.agents.read().await;
            let agents_list: Vec<_> = agents_guard.values().cloned().collect();
            let _ = sync_socket.emit("active_agents", serde_json::json!({ "agents": agents_list }));
            let _ = sync_socket.emit("socket_status", true);
        });

        // Hardware Update Event
        socket.on("pi_hardware_update", move |socket: SocketRef, Data(data): Data<serde_json::Value>| {
            let s_inner = s.clone();
            async move {
                let mut registry = s_inner.hardware_registry.write().await;
                registry.insert(socket.id.to_string(), data);
                let update: Vec<_> = registry.values().cloned().collect();
                s_inner.socket_manager.broadcast_hardware_update(serde_json::json!(update));
            }
        });

        // Behaviour Tree Init
        let lib = passform_agent_backend::behaviour_tree::skills::SkillLibrary::from_workspace();
        // Skills jetzt über Socket.io ans Elm Frontend sendbar
        let skills_json = serde_json::to_value(&lib.skills).unwrap_or_default();
        let _ = socket.emit("available_skills", skills_json);

        // Disconnect Event
        let dc_state = state.clone();
        socket.on_disconnect(move |socket: SocketRef| {
            let s_dc = dc_state.clone();
            async move {
                let mut registry = s_dc.hardware_registry.write().await;
                if registry.remove(&socket.id.to_string()).is_some() {
                    let update: Vec<_> = registry.values().cloned().collect();
                    s_dc.socket_manager.broadcast_hardware_update(serde_json::json!(update));
                }
            }
        });
    });
}

// --- TAURI COMMANDS ---

#[tauri::command]
async fn get_system_status(state: tauri::State<'_, Arc<AppState>>) -> Result<serde_json::Value, String> {
    let mode = state.infra_config.settings.read().await.current_mode.clone();
    Ok(serde_json::json!({ "status": "active", "mode": mode, "ros": "connected" }))
}

// --- MAIN ---

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();
    info!("🚀 PassForm 2 Core & Tauri App starten...");

    // 1. Pfade & Ressourcen-Setup
    // Wir definieren den Pfad zum Workspace relativ zum Ausführungsort
    let resource_path = PathBuf::from("../passform2_ws/src/passform_agent_resources");
    let resource_manager = Arc::new(ResourceManager::new(resource_path.clone()));

    // 2. ROS 2 Initialisierung
    let ros_context = Arc::new(rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?);
    let ros_client = Arc::new(RosClient::new(&ros_context)?);

    // 3. Manager & Layer Setup
    let (layer, io) = SocketIo::builder().build_layer();
    let socket_manager = Arc::new(SocketIoManager::new(io.clone()));
    
    // AgentManager bekommt jetzt den ResourceManager injiziert
    let agent_manager = Arc::new(AgentManager::new(
        socket_manager.clone(), 
        resource_manager.clone()
    ));
    
    let shared_state = Arc::new(AppState {
        hardware_registry: RwLock::new(HashMap::new()),
        agent_manager: agent_manager.clone(),
        infra_config: Arc::new(InfraConfigManager::new().expect("Config Error")),
        data_config: Arc::new(DataConfigManager::new()),
        http_client: HttpClient::new(),
        socket_manager,
        // Resource Manager in AppState aufnehmen, falls noch nicht geschehen
        resource_manager: resource_manager.clone(), 
        skill_manager: Arc::new(SkillManager::new()),
        path_manager: Arc::new(PathManager::new()),
        node_manager: Arc::new(NodeManager::new()),
        ros_client: ros_client.clone(),
    });

    // 4.1 Skill Library über ResourceManager laden
    // Anstatt hartkodiert "from_workspace" zu nutzen, ...
    let skill_lib = SkillLibrary::from_path(&resource_manager.skills_path);
    info!("Total Skills in Library: {}", skill_lib.skills.len());

    // let mut test_node = SkillNode::new(first_skill.clone());

    // TEST: Den ersten Skill mal "an-ticken" zum Testen
    if let Some(first_skill) = skill_lib.skills.first() {
        let mut test_node = SkillNode::new(first_skill.clone());
        test_node.tick().await; // Das sollte "🤖 BT Tick -> Skill: ..." loggen
    }
    
    if let Some(complex_skill) = skill_lib.skills.iter().find(|s| s.skill_type == "CUSTOM") {
    let mut root_node = passform_agent_backend::behaviour_tree::build_tree(complex_skill, &skill_lib);
        
        loop {
            let status = root_node.tick().await;
            if status != NodeStatus::Running {
                info!("🏁 BT Test-Durchlauf fertig mit Status: {:?}", status);
                break;
            }
            // Kurze Pause für die Lesbarkeit im Terminal
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        }
    }

    // 5. Tauri App Run
    let tauri_state = shared_state.clone();
    
    tauri::Builder::default()
        .manage(tauri_state) // State für Tauri Commands verfügbar machen
        .invoke_handler(tauri::generate_handler![get_system_status])
        .on_window_event(move |window, event| {
            if let tauri::WindowEvent::CloseRequested { .. } = event {
                let s = shared_state.clone();
                // Cleanup beim Schließen
                tokio::spawn(async move {
                    warn!("🛑 Shutdown eingeleitet...");
                    s.ros_client.stop();
                    s.node_manager.kill_all_nodes().await;
                });
            }
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");

    Ok(())
}

// --- HTTP HANDLER ---

async fn root_handler() -> Json<serde_json::Value> {
    Json(serde_json::json!({ "service": "PassForm 2 Tauri Core", "status": "active" }))
}

async fn get_agents(AxumState(state): AxumState<Arc<AppState>>) -> Json<Vec<serde_json::Value>> {
    let agents_guard = state.agent_manager.agents.read().await;
    let list = agents_guard.values().map(|a| serde_json::to_value(a).unwrap_or_default()).collect();
    Json(list)
}