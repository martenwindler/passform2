use std::collections::HashSet;
use std::sync::Arc;
use serde::{Deserialize, Serialize};
use tokio::sync::RwLock;
use tracing::{info, error, warn};
// Fix 1: Correcte syntaxis voor import alias
 
use axum::{extract::State, http::StatusCode, Json, response::IntoResponse};
use socketioxide::SocketIo;

// We gebruiken de ROS-Messages voor de mapping
use passform_agent_resources::msg::NavPathRequest as RosPathRequest;

// --- DOMAIN MODELS (API / Web) ---

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Position {
    pub x: i32,
    pub y: i32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PlanPathRequest {
    pub request_id: String,
    pub agent_id: String, 
    pub start: Position,
    pub goal: Position,
    #[serde(default)]
    pub central: bool,
}

#[derive(Serialize)]
pub struct PlanPathResponse {
    pub success: bool,
    pub message: String,
    pub request_id: String,
}

// --- MANAGER ---

pub struct PathManager {
    pub pending_requests: RwLock<HashSet<String>>,
}

impl PathManager {
    pub fn new() -> Self {
        info!("✅ PathManager Orchestrator fardig initialiseert.");
        Self {
            pending_requests: RwLock::new(HashSet::new()),
        }
    }

    pub async fn request_path(
        &self, 
        req: PlanPathRequest, 
        ros_client: Arc<crate::ros::ros_client::RosClient> 
    ) -> bool {
        let mut pending = self.pending_requests.write().await;
        pending.insert(req.request_id.clone());
        
        // --- MAPPING: API-Request -> ROS-Message ---
        // Fix 2: Gebruik de velden die de ROS-generator heeft aangemaakt (start_x, start_y, etc.)
        let ros_req = RosPathRequest {
            start_x: req.start.x,
            start_y: req.start.y,
            goal_x: req.goal.x,
            goal_y: req.goal.y,
            // Let op: De compiler gaf aan dat agent_id NIET in NavPathRequest.msg staat.
            // Als je agent_id nodig hebt, moet je de .msg vijle aanpassen en colcon build doen.
        };

        // Nu geven we het ROS-bericht (ros_req) door
        match ros_client.publish_path_request(ros_req) {
            Ok(_) => {
                info!("🗺️ Pfadplanung bi ROS angefordert: ID={} ({} -> {})", 
                    req.request_id, 
                    format!("x:{}, y:{}", req.start.x, req.start.y),
                    format!("x:{}, y:{}", req.goal.x, req.goal.y)
                );
                true
            },
            Err(e) => {
                error!("❌ ROS Fehler bi Pfadanfraag {}: {:?}", req.request_id, e);
                pending.remove(&req.request_id);
                false
            }
        }
    }

    pub async fn handle_path_complete(
        &self, 
        path_data: serde_json::Value, 
        io: &SocketIo
    ) {
        let req_id = path_data.get("request_id")
            .and_then(|id| id.as_str())
            .unwrap_or("unknown");

        let mut pending = self.pending_requests.write().await;
        
        if pending.remove(req_id) {
            info!("✅ Pfadplanung abgeschlossen för ID: {}", req_id);
        } else {
            warn!("⚠️ Empfange Pfad för nich registreerte ID: {}", req_id);
        }

        io.emit("path_complete", path_data).ok();
    }
}

// --- AXUM ROUTER ENDPUNKTE ---

pub async fn api_plan_path(
    State(state): State<Arc<crate::AppState>>, 
    Json(payload): Json<PlanPathRequest>,
) -> impl IntoResponse {
    let success = state.path_manager.request_path(payload.clone(), state.ros_client.clone()).await;

    if success {
        (
            StatusCode::OK,
            Json(PlanPathResponse {
                success: true,
                message: format!("Anfrage '{}' an ROS gesendet.", payload.request_id),
                request_id: payload.request_id,
            }),
        )
    } else {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(PlanPathResponse {
                success: false,
                message: "ROS-Dienst fehlerhaft oder niet erreichbaar.".to_string(),
                request_id: payload.request_id,
            }),
        )
    }
}