use axum::{extract::State, routing::{get, any, post}, Json, Router};
use socketioxide::{extract::{Data, SocketRef}, SocketIo};
use std::sync::Arc;
use std::net::SocketAddr;
use tracing::{info, error, warn};
use tower_http::{services::ServeDir, cors::{Any, CorsLayer}};
use reqwest::Client as HttpClient;
use tokio::sync::RwLock;
use std::collections::HashMap;

// Library-Importe aus dem Root
use passform_agent_backend::{
    AppState, 
    SocketIoManager, 
    SkillManager, 
    PathManager, 
    NodeManager, 
    AgentManager,
    InfraConfigManager,
    DataConfigManager
};
use passform_agent_backend::ros::ros_client::RosClient;
use passform_agent_backend::managers::path_manager::api_plan_path;

mod com;
use crate::com::proxy::aas_proxy_handler;

// --- SOCKET.IO HANDLER ---

fn setup_socket_handlers(io: &SocketIo, state: Arc<AppState>) {
    io.ns("/", move |socket: SocketRef, Data(_data): Data<serde_json::Value>| {
        let s = state.clone();
        info!("🔗 Socket verbunden: {} | Gesamt-Clients: ?", socket.id);

        // --- INITIAL SYNC (BEGRÜSSUNG) ---
        // Dies ersetzt den Python _handle_new_connection Callback
        let sync_state = s.clone();
        let sync_socket = socket.clone();
        
        tokio::spawn(async move {
            // 1. Modus sofort senden
            let mode = sync_state.infra_config.settings.read().await.current_mode.clone();
            let _ = sync_socket.emit("mode", mode);

            // 2. Aktuelle Agentenliste aus SSoT senden
            let agents_guard = sync_state.agent_manager.agents.read().await;
            let agents_list: Vec<_> = agents_guard.values().cloned().collect();
            let _ = sync_socket.emit("active_agents", serde_json::json!({ "agents": agents_list }));

            // 3. Status-Lampe im Frontend auf Grün setzen
            let _ = sync_socket.emit("socket_status", true);
            
            info!("✅ Initial-Sync für Client {} abgeschlossen.", sync_socket.id);
        });

        // --- EVENT: HARDWARE UPDATES ---
        socket.on("pi_hardware_update", move |socket: SocketRef, Data(data): Data<serde_json::Value>| {
            let s_inner = s.clone();
            async move {
                let mut registry = s_inner.hardware_registry.write().await;
                registry.insert(socket.id.to_string(), data.clone());
                
                let update: Vec<_> = registry.values().cloned().collect();
                s_inner.socket_manager.broadcast_hardware_update(serde_json::json!(update));
            }
        });

        // --- EVENT: CONFIG PUSH (Broadcast) ---
        socket.on("push_config", |socket: SocketRef, Data(data): Data<serde_json::Value>| async move {
            info!("⚙️ Config-Push von Client empfangen");
            let _ = socket.broadcast().emit("active_agents", data);
        });

        // --- EVENT: DISCONNECT ---
        socket.on_disconnect(move |socket: SocketRef| {
            let s_disconnect = state.clone();
            async move {
                let mut registry = s_disconnect.hardware_registry.write().await;
                if registry.remove(&socket.id.to_string()).is_some() {
                    warn!("❌ Hardware-Client getrennt: {}", socket.id);
                    let update: Vec<_> = registry.values().cloned().collect();
                    s_disconnect.socket_manager.broadcast_hardware_update(serde_json::json!(update));
                }
            }
        });
    });
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();
    info!("🚀 PassForm 2 Core startet (Jazzy)...");

    // 1. ROS 2 Kontext
    let ros_context = Arc::new(rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?);

    // 2. Infrastruktur & Manager-Initialisierung
    let (layer, io) = SocketIo::builder().build_layer();
    let socket_manager = Arc::new(SocketIoManager::new(io.clone()));
    
    let ros_client = Arc::new(RosClient::new(&ros_context)?);
    let skill_manager = Arc::new(SkillManager::new());
    let path_manager = Arc::new(PathManager::new());
    let node_manager = Arc::new(NodeManager::new());
    let agent_manager = Arc::new(AgentManager::new(socket_manager.clone())); 
    
    let infra_config = Arc::new(InfraConfigManager::new().expect("InfraConfig konnte nicht geladen werden"));
    let data_config = Arc::new(DataConfigManager::new());

    // 3. Shared State
    let shared_state = Arc::new(AppState {
        hardware_registry: RwLock::new(HashMap::new()),
        agent_manager: agent_manager.clone(),
        infra_config,
        data_config,
        http_client: HttpClient::new(),
        socket_manager: socket_manager.clone(),
        skill_manager: skill_manager.clone(),
        path_manager: path_manager.clone(),
        node_manager: node_manager.clone(),
        ros_client: ros_client.clone(),
    });

    // 4. ROS 2 Bridge Task
    let am_run = agent_manager.clone();
    let ros_client_run = ros_client.clone();
    tokio::spawn(async move {
        info!("📡 ROS 2 Bridge Task aktiv.");
        if let Err(e) = ros_client_run.run(am_run).await {
            error!("❌ ROS 2 Bridge kritischer Fehler: {:?}", e);
        }
    });

    // 5. Axum Router & Middleware
    setup_socket_handlers(&io, shared_state.clone());
    let cors = CorsLayer::new().allow_origin(Any).allow_methods(Any).allow_headers(Any);

    let app = Router::new()
        .route("/", get(root_handler))
        .route("/health", get(|| async { "OK" }))
        .route("/api/agents", get(get_agents))
        .route("/api/plan-path", post(api_plan_path))
        .route("/aas-api/*path", any(aas_proxy_handler))
        .nest_service("/basyx", ServeDir::new("static/basyx-ui").fallback(
            tower_http::services::ServeFile::new("static/basyx-ui/index.html")
        ))
        .layer(layer)
        .layer(cors)
        .with_state(shared_state.clone());

    // 6. Server Start mit Graceful Shutdown
    let addr = SocketAddr::from(([127, 0, 0, 1], 8080));
    let listener = tokio::net::TcpListener::bind(addr).await?;
    info!("⚓ Server läuft auf http://{}", addr);
    info!("🌐 BaSyx Dashboard unter http://{}/basyx/", addr);

    let shutdown_state = shared_state.clone();

    axum::serve(listener, app).with_graceful_shutdown(async move {
        tokio::signal::ctrl_c().await.ok();
        warn!("🛑 Shutdown-Signal empfangen! Räume auf...");
        shutdown_state.ros_client.stop();
        shutdown_state.node_manager.kill_all_nodes().await;
        info!("👋 Alles sauber beendet. Backend stoppt.");
    }).await?;

    Ok(())
}

// --- HTTP HANDLER ---

async fn root_handler() -> Json<serde_json::Value> {
    Json(serde_json::json!({ 
        "service": "PassForm 2 Core", 
        "ros_jazzy": "connected",
        "status": "active"
    }))
}

async fn get_agents(State(state): State<Arc<AppState>>) -> Json<Vec<serde_json::Value>> {
    let agents_guard = state.agent_manager.agents.read().await;
    let agents_list: Vec<serde_json::Value> = agents_guard.values()
        .map(|a| serde_json::to_value(a).unwrap_or(serde_json::json!({})))
        .collect();
    Json(agents_list)
}