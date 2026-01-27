use axum::{
    body::Body,
    extract::{Path, Request, State},
    response::Response,
    http::StatusCode,
};
use std::sync::Arc;
use crate::AppState;
use futures_util::TryStreamExt;
use tracing::{info, error};

pub async fn aas_proxy_handler(
    State(state): State<Arc<AppState>>,
    Path(path): Path<String>,
    req: Request,
) -> Result<Response<Body>, StatusCode> {
    
    info!("🔍 Proxy Request für Pfad: {}", path);

    // --- MOCK-LOGIK FÜR INFRASTRUKTUR (Verhindert 502 Fehler) ---
    // Wir fangen Registry, Discovery und Beschreibungen ab
    if path.contains("description") || path.contains("registry") || path.contains("discovery") {
        return Ok(Response::builder()
            .status(StatusCode::OK)
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"status": "Mocked by Rust Proxy"}"#))
            .unwrap());
    }

    // Wenn nach den Shells (Agenten) gefragt wird
    if path.contains("shells") && req.method() == axum::http::Method::GET {
        return handle_mock_shells(state).await;
    }

    // --- NORMALER PROXY (Weiterleitung an Docker auf 8081) ---
    let query = req.uri().query().map(|q| format!("?{}", q)).unwrap_or_default();
    let target_url = format!("http://localhost:8081/{}{}", path, query);
    
    let method = req.method().clone();
    let headers = req.headers().clone();

    match state.http_client
        .request(method, target_url)
        .headers(headers)
        .body(reqwest::Body::wrap_stream(req.into_body().into_data_stream()))
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
            let stream = res.bytes_stream().map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e));
            Ok(response_builder.body(Body::from_stream(stream)).unwrap())
        },
        Err(_) => {
            // Wenn Docker nicht läuft, liefern wir für Shells trotzdem unsere Liste
            if path.contains("shells") {
                handle_mock_shells(state).await
            } else {
                // Ansonsten geben wir jetzt ein leeres JSON statt 502 zurück
                Ok(Response::builder()
                    .status(StatusCode::OK)
                    .header("Content-Type", "application/json")
                    .body(Body::from("{}"))
                    .unwrap())
            }
        }
    }
}

async fn handle_mock_shells(state: Arc<AppState>) -> Result<Response<Body>, StatusCode> {
    let registry = state.hardware_registry.read().await;
    let mut shells = Vec::new();
    
    for (socket_id, data) in registry.iter() {
        let pi_name = data.get("pi_id").and_then(|v| v.as_str()).unwrap_or(socket_id);
        shells.push(serde_json::json!({
            "idShort": format!("Agent_{}", pi_name),
            "id": format!("http://passform.de/agents/{}", pi_name),
            "assetInformation": {
                "assetKind": "Instance",
                "globalAssetId": format!("asset-{}", pi_name)
            }
        }));
    }

    let json = serde_json::json!({
        "result": shells,
        "paging_metadata": {}
    });

    Ok(Response::builder()
        .status(StatusCode::OK)
        .header("Content-Type", "application/json")
        .body(Body::from(json.to_string()))
        .unwrap())
}