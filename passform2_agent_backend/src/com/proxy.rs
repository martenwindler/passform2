use axum::{
    body::Body, // Hier lag der Fehler (vorher ax_body)
    extract::{Path, Request, State},
    response::Response,
    http::{StatusCode, Method},
};
use std::sync::Arc;
use crate::AppState;
use tracing::info;
use serde_json::json;

#[allow(dead_code)]
pub async fn aas_proxy_handler(
    State(state): State<Arc<AppState>>,
    Path(path): Path<String>,
    req: Request,
) -> Result<Response<Body>, StatusCode> {
    
    info!("🔍 BaSyx Proxy Request: /{}", path);

    // 1. INFRASTRUKTUR-MOCKS
    if path.contains("description") {
        return Ok(Response::builder()
            .status(StatusCode::OK)
            .header("Content-Type", "application/json")
            .body(Body::from(json!({
                "identification": {"id": "passform-bridge-01"},
                "assetKind": "Instance"
            }).to_string()))
            .unwrap());
    }

    // 2. SHELLS ABFRAGE (Geändert von == auf .contains)
    // Wir prüfen auf "shell", um "shells" UND "shell-descriptors" zu erwischen
    if path.contains("shell") && req.method() == Method::GET {
        return handle_mock_shells(state).await;
    }

    // 3. NORMALER PROXY (Versuch der Weiterleitung an Docker/BaSyx-Server)
    let query = req.uri().query().map(|q| format!("?{}", q)).unwrap_or_default();
    let target_url = format!("http://localhost:8081/{}{}", path, query);
    
    let method = req.method().clone();
    let headers = req.headers().clone();

    // Wir extrahieren den Stream für reqwest
    let body_bytes = axum::body::to_bytes(req.into_body(), 1024 * 1024).await.unwrap_or_default();

    match state.http_client
        .request(method, target_url)
        .headers(headers)
        .body(body_bytes)
        .send()
        .await 
    {
        Ok(res) => {
            let mut response_builder = Response::builder().status(res.status());
            if let Some(headers_mut) = response_builder.headers_mut() {
                for (name, value) in res.headers() {
                    headers_mut.insert(name.clone(), value.clone());
                }
            }
            let body_content = res.bytes().await.unwrap_or_default();
            Ok(response_builder.body(Body::from(body_content)).unwrap())
        },
        Err(_) => {
            // Fallback: Wenn kein echter AAS-Server läuft, bedienen wir die Shells selbst
            if path.contains("shells") || path.contains("aas") {
                handle_mock_shells(state).await
            } else {
                Ok(Response::builder()
                    .status(StatusCode::NOT_FOUND)
                    .header("Content-Type", "application/json")
                    .body(Body::from(json!({"message": "Service unavailable, fallback empty"}).to_string()))
                    .unwrap())
            }
        }
    }
}

#[allow(dead_code)]
async fn handle_mock_shells(state: Arc<AppState>) -> Result<Response<Body>, StatusCode> {
    let mut shells = Vec::new();

    // TEIL A: Echte ROS-Agenten aus dem AgentManager (Priorität!)
    let agents = state.agent_manager.agents.read().await;
    for (id, agent) in agents.iter() {
        shells.push(json!({
            "idShort": id,
            "id": format!("http://passform.de/agents/{}", id),
            "assetInformation": {
                "assetKind": "Instance",
                "globalAssetId": format!("asset-{}", id)
            },
            "submodels": [
                {
                    "idShort": "Position",
                    "id": format!("http://passform.de/agents/{}/submodels/position", id),
                    "submodelElements": [
                        {"idShort": "x", "modelType": "Property", "value": agent.x},
                        {"idShort": "y", "modelType": "Property", "value": agent.y},
                        {"idShort": "orientation", "modelType": "Property", "value": agent.orientation}
                    ]
                },
                {
                    "idShort": "Status",
                    "submodelElements": [
                        {"idShort": "module_type", "modelType": "Property", "value": agent.module_type},
                        {"idShort": "status", "modelType": "Property", "value": agent.status}
                    ]
                }
            ]
        }));
    }

    // TEIL B: Hardware-Pis (Infrastruktur-Ebene)
    let registry = state.hardware_registry.read().await;
    for (socket_id, data) in registry.iter() {
        let pi_name = data.get("pi_id").and_then(|v| v.as_str()).unwrap_or(socket_id);
        // Wir fügen die Pis nur hinzu, wenn sie nicht schon als Agent existieren
        if !agents.contains_key(pi_name) {
            shells.push(json!({
                "idShort": format!("Pi_{}", pi_name),
                "id": format!("http://passform.de/hardware/{}", pi_name),
                "assetInformation": { "assetKind": "Instance" }
            }));
        }
    }

    let result = json!({
        "result": shells,
        "paging_metadata": {}
    });

    Ok(Response::builder()
        .status(StatusCode::OK)
        .header("Content-Type", "application/json")
        .header("Access-Control-Allow-Origin", "*")
        .body(Body::from(result.to_string()))
        .unwrap())
}