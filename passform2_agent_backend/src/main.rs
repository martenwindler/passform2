use axum::{
    extract::State,
    routing::{get, any},
    Json, Router,
};
use socketioxide::{
    extract::{Data, SocketRef},
    SocketIo,
};
use std::sync::Arc;
use tracing::info;
use tower_http::{
    services::ServeDir,
    cors::{Any, CorsLayer},
};
use reqwest::Client;
use tokio::sync::RwLock;
use std::collections::HashMap;

// Importiere die Typen aus unserer eigenen Library
use passform_agent_backend::{AppState, Agent}; 

mod com;
use crate::com::proxy::aas_proxy_handler;

// --- SOCKET.IO HANDLER ---
fn setup_socket_handlers(io: &SocketIo, state: Arc<AppState>) {
    io.ns("/", move |socket: SocketRef, Data(_data): Data<serde_json::Value>| {
        let s = state.clone();
        info!("🔗 Socket verbunden: {}", socket.id);

        socket.on("pi_hardware_update", move |socket: SocketRef, Data(data): Data<serde_json::Value>| {
            let s = s.clone();
            async move {
                let mut registry = s.hardware_registry.write().await;
                registry.insert(socket.id.to_string(), data.clone());
                let update: Vec<_> = registry.values().collect();
                socket.broadcast().emit("hardware_update", update).ok();
            }
        });

        socket.on("push_config", |socket: SocketRef, Data(data): Data<serde_json::Value>| async move {
            socket.broadcast().emit("active_agents", data).ok();
        });
    });
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();
    info!("🚀 PassForm Web-Backend startet...");

    // --- INITIALISIERUNG MIT DUMMY-DATEN ---
    let mut initial_hardware = HashMap::new();
    // Dieser Eintrag wird von der proxy.rs erkannt und als AAS angezeigt
    initial_hardware.insert(
        "test-socket-id-01".to_string(), 
        serde_json::json!({
            "pi_id": "Konstruktions-Bot-01",
            "status": "Online",
            "last_seen": chrono::Local::now().to_rfc3339()
        })
    );

    let shared_state = Arc::new(AppState {
        hardware_registry: RwLock::new(initial_hardware),
        agents: RwLock::new(Vec::new()),
        system_mode: RwLock::new("Simulation".to_string()),
        http_client: Client::new(), 
    });

    // --- SOCKET.IO SETUP ---
    let (layer, io) = SocketIo::builder().build_layer();
    setup_socket_handlers(&io, shared_state.clone());

    // --- CORS SETUP ---
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    // --- API ROUTER ---
    let app = Router::new()
        .route("/", get(root_handler))
        .route("/health", get(|| async { "OK" }))
        .route("/api/agents", get(get_agents))
        .route("/aas-api/*path", any(aas_proxy_handler))
        .nest_service("/basyx", ServeDir::new("static/basyx-ui"))
        .layer(layer)
        .layer(cors)
        .with_state(shared_state);

    // --- SERVER START ---
    let addr = "127.0.0.1:8080";
    let listener = tokio::net::TcpListener::bind(addr).await?;
    info!("⚓ Server läuft auf http://{}", addr);
    axum::serve(listener, app).await?;

    Ok(())
}

// --- HANDLER ---

async fn root_handler(State(state): State<Arc<AppState>>) -> Json<serde_json::Value> {
    let mode = state.system_mode.read().await;
    Json(serde_json::json!({ "service": "PassForm Proxy", "mode": *mode }))
}

async fn get_agents(State(state): State<Arc<AppState>>) -> Json<Vec<Agent>> {
    let agents = state.agents.read().await;
    Json(agents.clone())
}