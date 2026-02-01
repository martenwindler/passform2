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

    // 1. ROS 2 Initialisierung
    let ros_context = Arc::new(rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?);
    let ros_client = Arc::new(RosClient::new(&ros_context)?);

    // 2. Manager & Layer Setup
    let (layer, io) = SocketIo::builder().build_layer();
    let socket_manager = Arc::new(SocketIoManager::new(io.clone()));
    let agent_manager = Arc::new(AgentManager::new(socket_manager.clone()));
    
    let shared_state = Arc::new(AppState {
        hardware_registry: RwLock::new(HashMap::new()),
        agent_manager: agent_manager.clone(),
        infra_config: Arc::new(InfraConfigManager::new().expect("Config Error")),
        data_config: Arc::new(DataConfigManager::new()),
        http_client: HttpClient::new(),
        socket_manager,
        skill_manager: Arc::new(SkillManager::new()),
        path_manager: Arc::new(PathManager::new()),
        node_manager: Arc::new(NodeManager::new()),
        ros_client: ros_client.clone(),
    });

    // 3. Spawne ROS 2 Bridge Task
    let ros_am = agent_manager.clone();
    let ros_cl = ros_client.clone();
    tokio::spawn(async move {
        if let Err(e) = ros_cl.run(ros_am).await {
            error!("❌ ROS Bridge Fehler: {:?}", e);
        }
    });

    // 4. Spawne Axum Server Task
    let axum_state = shared_state.clone();
    setup_socket_handlers(&io, axum_state.clone());

    tokio::spawn(async move {
        let app = Router::new()
            .route("/", get(root_handler))
            .route("/api/agents", get(get_agents))
            .route("/api/plan-path", post(api_plan_path))
            .route("/aas-api/*path", any(aas_proxy_handler))
            .nest_service("/basyx", ServeDir::new("static/basyx-ui").fallback(
                ServeFile::new("static/basyx-ui/index.html")
            ))
            .layer(layer)
            .layer(CorsLayer::new().allow_origin(Any).allow_methods(Any).allow_headers(Any))
            .with_state(axum_state);

        let addr = SocketAddr::from(([127, 0, 0, 1], 8080));
        let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
        info!("⚓ Axum API & Socket.io auf http://{}", addr);
        axum::serve(listener, app).await.unwrap();
    });

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