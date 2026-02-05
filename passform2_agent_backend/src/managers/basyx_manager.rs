use axum::{
    body::Body,
    extract::{Path, Request, State},
    response::Response,
    http::{StatusCode, Method, header, HeaderMap, Uri},
};
use std::sync::Arc;
use tokio::sync::RwLock;
use crate::AppState;
use tracing::{info, warn, error};
use serde_json::{json, Value};

use crate::core::types::SkillType;
use crate::core::types::plant_model::PlantModel;
use crate::core::types::Bay;
use crate::core::types::location::{WorldLocation, LocalPoint, LocalPose};
use crate::core::types::module::Module;
use crate::core::types::inventory::WorldItem;

pub struct BasyxManager {
    pub plant: Arc<RwLock<PlantModel>>,
    pub aas_url: String,
}

impl BasyxManager {
    pub fn new(plant: Arc<RwLock<PlantModel>>, aas_url: String) -> Self {
        Self { plant, aas_url }
    }

    pub async fn register_plant(&self) -> Result<(), String> {
        let plant_guard = self.plant.read().await;
        let json_body = Self::render_plant_to_aas(&plant_guard);
        
        info!("📤 Sende PlantModel an BaSyx Server: {}", self.aas_url);

        let client = reqwest::Client::new();
        let resp = client
            .post(&self.aas_url)
            .json(&json_body)
            .send()
            .await
            .map_err(|e| e.to_string())?;

        if resp.status().is_success() {
            Ok(())
        } else {
            Err(format!("BaSyx Fehler: {}", resp.status()))
        }
    }

    // --- UNIVERSAL RENDERER FÜR DUMB DATA TYPES ---

    /// Rendert eine Bay (Bucht) inklusive ihrer Submodelle
    pub fn render_bay(bay: &Bay) -> Value {
        let category = if bay.is_virtual { "virtual_bay" } else { "passform_bay" };

        json!({
            "idShort": bay.name,
            "identification": { "id": bay.unique_id, "idType": "CUSTOM" },
            "category": category,
            "modelType": "SubmodelElementCollection",
            "submodels": [
                Self::render_bay_status(bay),
                Self::render_bay_technical_data(bay)
            ]
        })
    }

    fn render_bay_status(bay: &Bay) -> Value {
        json!({
            "idShort": "BayStatus",
            "modelType": "Submodel",
            "submodelElements": [
                { "idShort": "occupation", "modelType": "Property", "valueType": "boolean", "value": bay.occupation },
                { "idShort": "module", "modelType": "Property", "valueType": "string", "value": bay.module_uuid },
                { "idShort": "timeout", "modelType": "Property", "valueType": "boolean", "value": bay.has_timeout }
            ]
        })
    }

    fn render_bay_technical_data(bay: &Bay) -> Value {
        let mut elements = vec![
            json!({
                "idShort": "Origin",
                "modelType": "SubmodelElementCollection",
                "value": [
                    { "idShort": "x", "modelType": "Property", "value": bay.origin[0] },
                    { "idShort": "y", "modelType": "Property", "value": bay.origin[1] },
                    { "idShort": "z", "modelType": "Property", "value": bay.origin[2] }
                ]
            })
        ];

        if !bay.is_virtual {
            elements.push(json!({
                "idShort": "SupplyStatus",
                "modelType": "SubmodelElementCollection",
                "value": [
                    { "idShort": "dc_24v", "modelType": "Property", "value": true },
                    { "idShort": "ac_230v", "modelType": "Property", "value": false }
                ]
            }));
        }

        json!({
            "idShort": "TechnicalData",
            "modelType": "Submodel",
            "submodelElements": elements
        })
    }

    pub fn render_point(p: &LocalPoint) -> Value {
        json!({
            "idShort": "point",
            "modelType": "SubmodelElementCollection",
            "value": [
                { "idShort": "x", "modelType": "Property", "valueType": "double", "value": p.x },
                { "idShort": "y", "modelType": "Property", "valueType": "double", "value": p.y },
                { "idShort": "z", "modelType": "Property", "valueType": "double", "value": p.z }
            ]
        })
    }

    pub fn render_pose(pose: &LocalPose) -> Value {
        json!({
            "idShort": "pose",
            "modelType": "SubmodelElementCollection",
            "value": [
                Self::render_point(&pose.position),
                {
                    "idShort": "quaternion",
                    "modelType": "SubmodelElementCollection",
                    "value": [
                        { "idShort": "x", "modelType": "Property", "value": pose.orientation.x },
                        { "idShort": "y", "modelType": "Property", "value": pose.orientation.y },
                        { "idShort": "z", "modelType": "Property", "value": pose.orientation.z },
                        { "idShort": "w", "modelType": "Property", "value": pose.orientation.w }
                    ]
                }
            ]
        })
    }

    pub fn render_location(loc: &WorldLocation) -> Value {
        let polygon_elements: Vec<Value> = loc.aoi.points.iter().enumerate().map(|(i, p)| {
            json!({
                "idShort": format!("point_{}", i),
                "modelType": "SubmodelElementCollection",
                "value": [
                    { "idShort": "x", "modelType": "Property", "value": p.x },
                    { "idShort": "y", "modelType": "Property", "value": p.y },
                    { "idShort": "z", "modelType": "Property", "value": p.z }
                ]
            })
        }).collect();

        json!({
            "idShort": "location",
            "modelType": "SubmodelElementCollection",
            "value": [
                { "idShort": "frame_id", "modelType": "Property", "value": loc.frame_id },
                {
                    "idShort": "area_of_interest",
                    "modelType": "SubmodelElementCollection",
                    "value": [
                        { "idShort": "label", "modelType": "Property", "value": loc.aoi.label },
                        { "idShort": "uid", "modelType": "Property", "value": loc.aoi.uid },
                        { "idShort": "polygon", "modelType": "SubmodelElementCollection", "value": polygon_elements }
                    ]
                },
                Self::render_pose(&loc.pose)
            ]
        })
    }

    pub fn render_module_to_aas(module: &Module) -> Value {
        let mut tech_elements = Vec::new();

        let mut supply_values = Vec::new();
        if module.supply.dc_24v { supply_values.push(json!({"idShort": "dc_24v", "modelType": "Property", "value": true})); }
        if module.supply.ac_230v { supply_values.push(json!({"idShort": "ac_230v", "modelType": "Property", "value": true})); }
        if module.supply.vacuum { supply_values.push(json!({"idShort": "vacuum", "modelType": "Property", "value": true})); }

        tech_elements.push(json!({
            "idShort": "SupplyCharacteristics",
            "modelType": "SubmodelElementCollection",
            "value": supply_values
        }));

        json!({
            "idShort": module.name,
            "identification": { "id": module.uuid, "idType": "CUSTOM" },
            "modelType": "AssetAdministrationShell",
            "submodels": [
                {
                    "idShort": "TechnicalData",
                    "modelType": "Submodel",
                    "submodelElements": tech_elements
                },
                {
                    "idShort": "inventory",
                    "modelType": "Submodel",
                    "submodelElements": module.inventory.clone()
                }
            ]
        })
    }

    pub fn render_item(item: &WorldItem) -> Value {
        use crate::core::util::util::sanitize_id;
        let id_short = format!("item_{}", sanitize_id(&item.uid));
        
        json!({
            "idShort": id_short,
            "category": "WorldItem",
            "modelType": "SubmodelElementCollection",
            "value": [
                { "idShort": "name", "modelType": "Property", "valueType": "string", "value": item.name },
                { "idShort": "uid", "modelType": "Property", "valueType": "string", "value": item.uid },
                { "idShort": "quantity", "modelType": "Property", "valueType": "integer", "value": item.quantity },
                Self::render_location(&item.location)
            ]
        })
    }

    pub fn render_plant_to_aas(plant: &PlantModel) -> serde_json::Value {
        let submodels: Vec<serde_json::Value> = plant.bays
            .iter()
            .map(|b: &Bay| Self::render_bay(b)) 
            .collect();

        serde_json::json!({
            "idShort": plant.name,
            "identification": { "id": plant.unique_id, "idType": "CUSTOM" },
            "category": "passform_master",
            "modelType": "AssetAdministrationShell",
            "asset": {
                "idShort": "MasterAsset",
                "identification": { "id": format!("{}_ASSET", plant.unique_id), "idType": "CUSTOM" },
                "assetKind": "INSTANCE"
            },
            "submodels": submodels
        })
    }

    // --- PROXY & INFRASTRUKTUR LOGIK ---

    pub async fn proxy_handler(
        State(state): State<Arc<AppState>>,
        path: Option<Path<String>>,
        req: Request, 
    ) -> Result<Response<Body>, StatusCode> {
        let path_str = path.map(|p| p.0).unwrap_or_default();
        let full_uri = req.uri().path().to_string();
        
        info!("🔍 BaSyx Request: path_str='{}', full_uri='{}'", path_str, full_uri);

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

        if path_str.is_empty() || (path_str.contains('.') && !is_api_request) {
            return Self::serve_static(&path_str).await;
        }

        let method = req.method().clone();
        let uri = req.uri().clone();
        let headers = req.headers().clone();
        let body = req.into_body();

        match Self::forward_to_docker(&state, &path_str, method, headers, uri, body).await {
            Ok(resp) => Ok(resp),
            Err(_) => Ok(Self::error_response("BaSyx Infrastructure (Docker) not reachable")),
        }
    }

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

    async fn handle_virtual_shells(state: Arc<AppState>) -> Response<Body> {
        let mut shells = Vec::new();
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