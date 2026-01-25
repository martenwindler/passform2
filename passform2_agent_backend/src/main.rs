use axum::{extract::State, routing::get, Json, Router};
use socketioxide::{extract::{Data, SocketRef}, SocketIo};
use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use tracing::{info, error};
use tokio::sync::RwLock;

// --- DOMAIN & REPRESENTATIONS ---
#[derive(Serialize, Deserialize, Clone, Debug)]
struct Agent {
    agent_id: String,
    x: i32,
    y: i32,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
struct HardwareInfo {
    pi_id: String,
    status: String,
}

// --- SHARED STATE (De SSoT in'n RAM) ---
struct AppState {
    hardware_registry: RwLock<HashMap<String, serde_json::Value>>,
    agents: RwLock<Vec<Agent>>,
    system_mode: RwLock<String>,
}

// --- SOCKET.IO HANDLER ---
async fn on_connect(socket: SocketRef, Data(data): Data<serde_json::Value>) {
    info!("🔗 Socket verbunden: {}", socket.id);
    
    // pi_hardware_update
    socket.on("pi_hardware_update", |socket: SocketRef, Data(data): Data<serde_json::Value>, state: State<Arc<AppState>>| async move {
        let mut registry = state.hardware_registry.write().await;
        registry.insert(socket.id.to_string(), data.clone());
        info!("🖥️ Pi Hardware Update: {:?}", data.get("pi_id"));
        let update: Vec<_> = registry.values().collect();
        socket.broadcast().emit("hardware_update", update).ok();
    });

    // push_config
    socket.on("push_config", |socket: SocketRef, Data(data): Data<serde_json::Value>, state: State<Arc<AppState>>| async move {
        if let Some(agents_raw) = data.get("agents") {
            info!("📦 BACKEND: Erhaltene Agenten-Anzahl: {:?}", agents_raw.as_array().map(|a| a.len()));
            // Hier würdest du config_manager.save_to_ssot() aufrufen
            socket.broadcast().emit("active_agents", data).ok();
        }
    });

    // plan_path (Placeholder für dein Planner-Modul)
    socket.on("plan_path", |socket: SocketRef, Data(data): Data<serde_json::Value>| async move {
        info!("🗺️ Pfadplanung angefragt");
        // Hier Aufruf von managers::path_manager::a_star
        socket.emit("path_complete", serde_json::json!({"status": 200, "path": []})).ok();
    });
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Logging initialisieren (Industrial Excellence)
    tracing_subscriber::fmt::init();
    info!("🚀 PassForm Backend startet in Rust...");

    // 2. Shared State aufbauen
    let shared_state = Arc::new(AppState {
        hardware_registry: RwLock::new(HashMap::new()),
        agents: RwLock::new(Vec::new()),
        system_mode: RwLock::new("Simulation".to_string()),
    });

    // 3. Socket.io Setup
    let (layer, io) = SocketIo::builder()
        .with_state(shared_state.clone())
        .build_layer();

    io.ns("/", on_connect);

    // 4. API Router (Axum)
    let app = Router::new()
        .route("/", get(root_handler))
        .route("/health", get(|| async { "OK" }))
        // Hier kommen die Router aus den Managern rein
        // .nest("/api/nodes", managers::node_manager::router())
        .layer(layer) // Socket.io Layer hinzufügen
        .with_state(shared_state);

    // 5. Startup-Logik (Ersatz für Lifespan)
    // Hier rufen wir die Logik auf, die früher im lifespan-Block stand
    info!("✅ Orchestrator: System-Nodes werden gestartet...");
    // managers::node_manager::start_system_nodes().await;

    // 6. Server Stapellauf
    let listener = tokio::net::TcpListener::bind("127.0.0.1:8080").await?;
    info!("⚓ PassForm Backend an http://127.0.0.1:8080 bereit");
    axum::serve(listener, app).await?;

    Ok(())
}

async fn root_handler(state: State<Arc<AppState>>) -> Json<serde_json::Value> {
    let mode = state.system_mode.read().await;
    Json(serde_json::json!({
        "service": "PassForm Backend (Rust Edition)",
        "mode": *mode,
        "engine": "Axum + Tokio"
    }))
}