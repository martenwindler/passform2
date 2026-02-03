#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use axum::{extract::State as AxumState, Json};
use socketioxide::{extract::{Data, SocketRef}, SocketIo};
use std::sync::Arc;
use tracing::{info, warn};
use reqwest::Client as HttpClient;
use tokio::sync::RwLock;
use std::collections::{HashMap, HashSet};

// Library-Importe aus dem Backend-Crate
use passform_agent_backend::{
    AppState, SocketIoManager, SkillManager, PathManager, 
    NodeManager, AgentManager, InfraConfigManager, DataConfigManager
};

use passform_agent_backend::ros::ros_client::RosClient;
use passform_agent_backend::managers::resource_manager::ResourceManager;
use std::path::PathBuf;

// Behavior Tree & HTN Planner Importe
use passform_agent_backend::behaviour_tree::skills::SkillLibrary;
use passform_agent_backend::behaviour_tree::skill_node::SkillNode;
use passform_agent_backend::behaviour_tree::{BehaviorNode, NodeStatus, WorldState, HTNPlanner};

mod com;

// --- SOCKET.IO HANDLER ---

fn setup_socket_handlers(io: &SocketIo, state: Arc<AppState>) {
    io.ns("/", move |socket: SocketRef| {
        let s = state.clone();
        info!("🔗 Socket verbunden: {} | Transport: {:?}", socket.id, socket.transport_type());

        // 1. Initial Sync
        let sync_state = s.clone();
        let sync_socket = socket.clone();
        tokio::spawn(async move {
            let mode = sync_state.infra_config.settings.read().await.current_mode.clone();
            let _ = sync_socket.emit("mode", mode);
            
            let agents_guard = sync_state.agent_manager.agents.read().await;
            let agents_list: Vec<_> = agents_guard.values().cloned().collect();
            let _ = sync_socket.emit("active_agents", serde_json::json!({ "agents": agents_list }));
            
            let _ = sync_socket.emit("hardware_specs", serde_json::json!(sync_state.resource_manager.hardware_specs));
            
            // Sync WorldState
            let current_world = sync_state.world_state.read().await;
            let _ = sync_socket.emit("world_state", serde_json::json!(current_world.0));
            
            let _ = sync_socket.emit("socket_status", true);
        });

        // 2. Hardware Update Event
        let s_for_update = s.clone(); 
        socket.on("pi_hardware_update", move |socket: SocketRef, Data(data): Data<serde_json::Value>| {
            let s_inner = s_for_update.clone();
            async move {
                let mut registry = s_inner.hardware_registry.write().await;
                registry.insert(socket.id.to_string(), data);
                let update: Vec<_> = registry.values().cloned().collect();
                let _ = s_inner.socket_manager.broadcast_hardware_update(serde_json::json!(update)).await; 
            }
        });

        // Skill Library & BT Sync
        let lib = SkillLibrary::from_path(&s.resource_manager.skills_path);
        let skills_json = serde_json::to_value(&lib.skills).unwrap_or_default();
        let _ = socket.emit("available_skills", skills_json);

        // 3. Disconnect Event
        let s_for_dc = s.clone();
        socket.on_disconnect(move |socket: SocketRef| {
            let s_dc = s_for_dc.clone();
            async move {
                let mut registry = s_dc.hardware_registry.write().await;
                if registry.remove(&socket.id.to_string()).is_some() {
                    let update: Vec<_> = registry.values().cloned().collect();
                    let _ = s_dc.socket_manager.broadcast_hardware_update(serde_json::json!(update)).await;
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
    let resource_path = PathBuf::from("../passform2_ws/src/passform_agent_resources");
    let resource_manager = Arc::new(ResourceManager::new(resource_path.clone()));

    // 2. ROS 2 Initialisierung
    let ros_context = Arc::new(rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?);
    let ros_client = Arc::new(RosClient::new(&ros_context)?);

    // 3. Manager & Layer Setup
    let (layer, io) = SocketIo::builder().build_layer();
    let socket_manager = Arc::new(SocketIoManager::new(io.clone()));
    
    let agent_manager = Arc::new(AgentManager::new(
        socket_manager.clone(), 
        resource_manager.clone()
    ));
    
    // Initialisierung des AppState mit WorldState und Planner
    let shared_state = Arc::new(AppState {
        hardware_registry: RwLock::new(HashMap::new()),
        agent_manager,
        infra_config: Arc::new(InfraConfigManager::new().expect("Config Error")),
        data_config: Arc::new(DataConfigManager::new()),
        http_client: HttpClient::new(),
        socket_manager,
        resource_manager: resource_manager.clone(), 
        skill_manager: Arc::new(SkillManager::new()),
        path_manager: Arc::new(PathManager::new()),
        node_manager: Arc::new(NodeManager::new()),
        ros_client: ros_client.clone(),
        // HTN & WorldState Integration
        world_state: RwLock::new(WorldState(HashSet::new())),
        planner: Arc::new(HTNPlanner {
            operators: HashMap::new(), // Wird später via ResourceManager befüllt
            methods: HashMap::new(),
        }),
    });

    // --- 4. Axum Server Task ---
    let axum_state = shared_state.clone();
    let io_layer = layer.clone(); 

    tokio::spawn(async move {
        let app = axum::Router::new()
            .route("/", axum::routing::get(root_handler))
            .route("/api/agents", axum::routing::get(get_agents))
            .route("/api/resources/specs", axum::routing::get(get_hardware_specs))
            .route("/api/resources/interfaces", axum::routing::get(get_ros_interfaces))
            .route("/api/planner/plan", axum::routing::post(handle_planning_request))
            .layer(io_layer)
            .with_state(axum_state);

        let addr = std::net::SocketAddr::from(([127, 0, 0, 1], 8080));
        let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
        
        info!("⚓ Axum API & Socket.io Server läuft auf http://{}", addr);
        axum::serve(listener, app).await.unwrap();
    });

    // 5. Skill Library & BT Test
    let skill_lib = SkillLibrary::from_path(&resource_manager.skills_path);
    info!("Total Skills in Library: {}", skill_lib.skills.len());

    if let Some(first_skill) = skill_lib.skills.first() {
        let mut test_node = SkillNode::new(first_skill.clone());
        let _ = test_node.tick().await; 
    }
    
    if let Some(complex_skill) = skill_lib.skills.iter().find(|s| s.skill_type == "CUSTOM") {
        let mut root_node = passform_agent_backend::behaviour_tree::build_tree(complex_skill, &skill_lib);
        
        tokio::spawn(async move {
            loop {
                let status = root_node.tick().await;
                if status != NodeStatus::Running {
                    info!("🏁 BT Test-Durchlauf fertig mit Status: {:?}", status);
                    break;
                }
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            }
        });
    }

    // 6. Tauri App Run
    setup_socket_handlers(&io, shared_state.clone());

    let tauri_state = shared_state.clone();
    tauri::Builder::default()
        .manage(tauri_state)
        .invoke_handler(tauri::generate_handler![get_system_status])
        .on_window_event(move |_window, event| {
            if let tauri::WindowEvent::CloseRequested { .. } = event {
                let s = shared_state.clone();
                tokio::spawn(async move {
                    warn!("🛑 Shutdown eingeleitet...");
                    s.ros_client.stop();
                    let _ = s.node_manager.kill_all_nodes().await;
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

async fn get_hardware_specs(AxumState(state): AxumState<Arc<AppState>>) -> Json<serde_json::Value> {
    Json(serde_json::json!(state.resource_manager.hardware_specs))
}

async fn get_ros_interfaces(AxumState(state): AxumState<Arc<AppState>>) -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "msgs": state.resource_manager.available_msgs,
        "srvs": state.resource_manager.available_srvs,
        "actions": state.resource_manager.available_actions,
    }))
}

async fn handle_planning_request(
    AxumState(state): AxumState<Arc<AppState>>,
    Json(payload): Json<serde_json::Value>
) -> Json<serde_json::Value> {
    let tasks = payload["tasks"].as_array()
        .map(|t| t.iter().filter_map(|v| v.as_str().map(|s| s.to_string())).collect())
        .unwrap_or_else(Vec::new);

    let current_state = state.world_state.read().await.clone();
    
    match state.planner.plan(current_state, tasks) {
        Ok(plan) => Json(serde_json::json!({ "status": "success", "plan": plan })),
        Err(e) => Json(serde_json::json!({ "status": "error", "message": e })),
    }
}