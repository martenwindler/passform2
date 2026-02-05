use axum::{
    body::Body,
    extract::{Path, Request, State},
    response::Response,
    http::{StatusCode, Method, header, HeaderMap, Uri},
};
use std::sync::Arc;
use crate::AppState;
use tracing::{info, warn};
use serde_json::{json, Value};

use crate::core::types::SkillType;

pub struct BasyxManager;

impl BasyxManager {
    /// Zentraler Einstiegspunkt für alle /basyx/* und /aas-api/* Anfragen.
    pub async fn proxy_handler(
        State(state): State<Arc<AppState>>,
        path: Option<Path<String>>,
        req: Request, 
    ) -> Result<Response<Body>, StatusCode> {
        
        let path_str = path.map(|p| p.0).unwrap_or_default();
        let full_uri = req.uri().path().to_string();
        
        info!("🔍 BaSyx Request: path_str='{}', full_uri='{}'", path_str, full_uri);

        // 1. AAS API ERKENNUNG (Priorität vor statischen Dateien)
        let is_api_request = full_uri.contains("aas-api") 
            || path_str.contains("shells") 
            || path_str.contains("shell-descriptors")
            || path_str.contains("submodels");

        if is_api_request && req.method() == Method::GET {
            match Self::forward_to_docker(&state, &path_str, Method::GET, req.headers().clone(), req.uri().clone(), Body::empty()).await {
                Ok(resp) if resp.status().is_success() => return Ok(resp),
                _ => {
                    warn!("⚠️ BaSyx Docker API offline - sende Virtual Shells.");
                    return Ok(Self::handle_virtual_shells(state).await);
                }
            }
        }

        // 2. STATISCHE UI (Dashboard Assets)
        if path_str.is_empty() || (path_str.contains('.') && !is_api_request) {
            return Self::serve_static(&path_str).await;
        }

        // 3. INFRASTRUKTUR MOCKS
        if path_str.contains("description") {
            return Ok(Self::json_response(json!({
                "identification": {"id": "passform-bridge-01"},
                "assetKind": "Instance"
            })));
        }

        // 4. GENERAL PROXY
        let method = req.method().clone();
        let uri = req.uri().clone();
        let headers = req.headers().clone();
        let body = req.into_body();

        match Self::forward_to_docker(&state, &path_str, method, headers, uri, body).await {
            Ok(resp) => Ok(resp),
            Err(_) => Ok(Self::error_response("BaSyx Infrastructure (Docker) not reachable")),
        }
    }

    /// Hilfsmethode: Erzeugt das BaSyx-JSON für einen Skill basierend auf dem SkillType Enum
    fn build_skill_submodel(skill: SkillType, driver: &str) -> Value {
        json!({
            "idShort": skill.as_str(),
            "modelType": "Submodel",
            "semanticId": { "keys": [{ "type": "GlobalReference", "value": "http://passform.de/semantics/skill" }] },
            "submodelElements": [
                {
                    "idShort": "skillType",
                    "modelType": "Property",
                    "valueType": "int",
                    "value": skill as i32
                },
                {
                    "idShort": "driverTopic",
                    "modelType": "Property",
                    "valueType": "string",
                    "value": driver
                },
                {
                    "idShort": "operations",
                    "modelType": "SubmodelElementCollection",
                    "value": [] 
                }
            ]
        })
    }

    /// Hilfsmethode: Erzeugt AAS-Shells dynamisch inklusive Skills aus dem AgentManager
    async fn handle_virtual_shells(state: Arc<AppState>) -> Response<Body> {
        let mut shells = Vec::new();
        
        // A: ROS Agenten mit dynamischen Skills
        let agents = state.agent_manager.agents.read().await;
        for (id, agent) in agents.iter() {
            let mut submodels = vec![
                json!({
                    "idShort": "DynamicData",
                    "submodelElements": [
                        {"idShort": "x", "modelType": "Property", "value": agent.x},
                        {"idShort": "y", "modelType": "Property", "value": agent.y},
                        {"idShort": "status", "modelType": "Property", "value": agent.status}
                    ]
                })
            ];

            // Skill-Injektion via SkillType Enum
            match agent.module_type.to_lowercase().as_str() {
                "ur5" => {
                    submodels.push(Self::build_skill_submodel(SkillType::AssemblyPickPlace, "/ur5/pick_place"));
                    submodels.push(Self::build_skill_submodel(SkillType::Release, "/ur5/release"));
                },
                "ranger" | "ftf" | "deebot" => {
                    submodels.push(Self::build_skill_submodel(SkillType::Move, &format!("/{}/move", id.to_lowercase())));
                },
                "human_operator" => {
                    submodels.push(Self::build_skill_submodel(SkillType::Stop, "/system/halt"));
                },
                _ => {}
            }

            shells.push(json!({
                "idShort": id,
                "id": format!("http://passform.de/agents/{}", id),
                "assetInformation": { "assetKind": "Instance" },
                "submodels": submodels
            }));
        }

        // B: Hardware Pis (Infrastruktur)
        let registry = state.hardware_registry.read().await;
        for (socket_id, data) in registry.iter() {
            let name = data.get("pi_id").and_then(|v| v.as_str()).unwrap_or(socket_id);
            if !agents.contains_key(name) {
                shells.push(json!({
                    "idShort": format!("Hardware_{}", name),
                    "id": format!("http://passform.de/pi/{}", name),
                    "assetInformation": { "assetKind": "Instance" },
                    "submodels": []
                }));
            }
        }

        Self::json_response(json!({ "result": shells, "paging_metadata": {} }))
    }

    // --- Private Hilfsmethoden für Proxy & Statics ---

    async fn forward_to_docker(state: &Arc<AppState>, path: &str, method: Method, headers: HeaderMap, uri: Uri, body: Body) -> Result<Response<Body>, reqwest::Error> {
        let query = uri.query().map(|q| format!("?{}", q)).unwrap_or_default();
        let target_url = format!("http://localhost:8081/{}{}", path, query);
        let stream = body.into_data_stream();
        state.http_client
            .request(method, target_url)
            .headers(headers)
            .body(reqwest::Body::wrap_stream(stream))
            .send().await
            .map(|res| {
                let mut rb = Response::builder().status(res.status());
                for (n, v) in res.headers() { rb = rb.header(n, v); }
                rb.body(Body::from_stream(res.bytes_stream())).unwrap()
            })
    }

    async fn serve_static(path: &str) -> Result<Response<Body>, StatusCode> {
        let clean_path = if path.is_empty() || path == "/" { "index.html" } else { path };
        let full_path = std::path::Path::new("static/basyx-ui").join(clean_path);
        match tokio::fs::read(&full_path).await {
            Ok(content) => {
                let mime = mime_guess::from_path(&full_path).first_or_octet_stream();
                Ok(Response::builder()
                    .header(header::CONTENT_TYPE, mime.as_ref())
                    .header(header::ACCESS_CONTROL_ALLOW_ORIGIN, "*")
                    .body(Body::from(content)).unwrap())
            },
            Err(_) => Err(StatusCode::NOT_FOUND)
        }
    }

    fn json_response(value: Value) -> Response<Body> {
        Response::builder()
            .header(header::CONTENT_TYPE, "application/json")
            .header(header::ACCESS_CONTROL_ALLOW_ORIGIN, "*")
            .body(Body::from(value.to_string())).unwrap()
    }

    fn error_response(msg: &str) -> Response<Body> {
        Response::builder().status(StatusCode::SERVICE_UNAVAILABLE)
            .header(header::CONTENT_TYPE, "application/json")
            .body(Body::from(json!({"error": msg, "fallback": "active"}).to_string())).unwrap()
    }
}