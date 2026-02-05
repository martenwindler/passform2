#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use axum::{extract::State as AxumState, Json, http::Method};
use socketioxide::{extract::{Data, SocketRef}, SocketIo, SocketIoConfig};
use std::sync::Arc;
use tracing::{info, warn, error};
use reqwest::Client as HttpClient;
use tokio::sync::RwLock;
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use tower_http::cors::{Any, CorsLayer};

// Library-Importe aus dem Backend-Crate (SSoT)
use passform_agent_backend::{
    AppState, PlantModel, SocketIoManager, SkillActionManager, PathManager, 
    NodeManager, AgentManager, InventoryManager, InfraConfigManager, DataConfigManager, MatchManager, RfidManager, SystemRole, SystemMode
};

use passform_agent_backend::core::util::watchdog::Watchdog;
use passform_agent_backend::ros::ros_client::RosClient;
use passform_agent_backend::managers::{
    resource_manager::ResourceManager,
    protocol_manager::ProtocolManager,
    system_api::SystemApi,
    basyx_manager::BasyxManager
};

// Behavior Tree & HTN Planner Importe
use passform_agent_backend::behaviour_tree::skills::SkillLibrary;
use passform_agent_backend::behaviour_tree::{NodeStatus, WorldState, HTNPlanner};

// --- SOCKET.IO HANDLER ---
fn setup_socket_handlers(io: &SocketIo, state: Arc<AppState>) {
    io.ns("/", move |socket: SocketRef| {
        let s = state.clone();
        info!("🔗 Socket verbunden: {} | Transport: {:?}", socket.id, socket.transport_type());

        // 1. Initial Sync
        let sync_state = s.clone();
        let sync_socket = socket.clone();
        tokio::spawn(async move {
            let mode = sync_state.infra_config.settings.read().await.current_mode;
            let _ = sync_socket.emit("mode", mode.to_string());
            
            let current_world = sync_state.world_state.read().await;
            let _ = sync_socket.emit("world_state", serde_json::json!(current_world.0));

            // In main.rs -> setup_socket_handlers -> Initial Sync Block
            let plant_guard = sync_state.plant_model.read().await;
            let _ = sync_socket.emit("initial_bays", serde_json::json!(plant_guard.bays));

            let agents_guard = sync_state.agent_manager.agents.read().await;
            let agents_list: Vec<_> = agents_guard.values().cloned().collect();
            let _ = sync_socket.emit("active_agents", serde_json::json!({ "agents": agents_list }));

            // Innerhalb von tokio::spawn im Socket-Handler:
            let items_guard = sync_state.inventory_manager.items.read().await;
            let items_list: Vec<_> = items_guard.values().cloned().collect();
            let _ = sync_socket.emit("inventory_update", serde_json::json!({ "items": items_list }));

            let _ = sync_socket.emit("hardware_specs", serde_json::json!(sync_state.resource_manager.hardware_specs));
            
            let _ = sync_socket.emit("socket_status", true);

        });

        // 2. Hardware Update Event
        let s_for_update = s.clone(); 
        socket.on("pi_hardware_update", move |socket: SocketRef, Data(data): Data<serde_json::Value>| {
            let s_inner = s_for_update.clone();
            let sid = socket.id.to_string();
            tokio::spawn(async move {
                let mut registry = s_inner.hardware_registry.write().await;
                registry.insert(sid, data);
                let update: Vec<_> = registry.values().cloned().collect();
                let _ = s_inner.socket_manager.io.emit("hardware_update", serde_json::json!(update));
            });
        });

        // Skill Library & BT Sync
        let lib = SkillLibrary::from_path(&s.resource_manager.skills_path);
        let skills_json = serde_json::to_value(&lib.skills).unwrap_or_default();
        let _ = socket.emit("available_skills", skills_json);

        // 3. Disconnect Event
        let s_for_dc = s.clone();
        socket.on_disconnect(move |socket: SocketRef| {
            let s_dc = s_for_dc.clone();
            let sid = socket.id.to_string();
            tokio::spawn(async move {
                let mut registry = s_dc.hardware_registry.write().await;
                if registry.remove(&sid).is_some() {
                    let update: Vec<_> = registry.values().cloned().collect();
                    let _ = s_dc.socket_manager.io.emit("hardware_update", serde_json::json!(update));
                }
            });
        });
    });
}

// --- TAURI COMMANDS ---
#[tauri::command]
async fn get_system_status(state: tauri::State<'_, Arc<AppState>>) -> Result<serde_json::Value, String> {
    let mode = state.infra_config.settings.read().await.current_mode;
    Ok(serde_json::json!({ "status": "active", "mode": mode.to_string(), "ros": "connected" }))
}

// --- MAIN ---
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();
    info!("🚀 PassForm 2 Core & Tauri App starten...");

    // 1. Infrastruktur-Config laden
    let infra_config = Arc::new(InfraConfigManager::new().expect("❌ Config Error"));
    let role = infra_config.get_role().await;

    // 2. Pfade & Ressourcen-Setup
    let resource_path = PathBuf::from("../passform2_ws/src/passform_agent_resources");
    let resource_manager = Arc::new(ResourceManager::new(resource_path.clone()));

    // 3. ROS 2 Initialisierung
    let ros_context = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
    let ros_client = Arc::new(RosClient::new(&ros_context)?);

    // 4. Manager & Layer Setup
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods([Method::GET, Method::POST])
        .allow_headers(Any);

    let (layer, io) = SocketIo::builder()
        .with_config(SocketIoConfig::default())
        .build_layer();

    let socket_manager = Arc::new(SocketIoManager::new(io.clone()));
    let system_api = Arc::new(SystemApi::new());
    let inventory_manager = Arc::new(InventoryManager::new(socket_manager.clone()));

    let mut plant = PlantModel::new(
        Some("plant_model_master_01".to_string()), 
        Some("PassForM_Production_Line".to_string())
    );

    // Beispielhafte Buchten hinzufügen (kann auch aus einer Config kommen)
    // Reihe 1: Positionen X: 1.0 bis 4.0 | Y: 1.0
    plant.create_bay("Bay-Tisch-1", [1.0, 1.0, 0.0]);
    plant.create_bay("Bay-Tisch-2", [2.0, 1.0, 0.0]);
    plant.create_bay("Bay-Tisch-3", [3.0, 1.0, 0.0]);
    plant.create_bay("Bay-Tisch-4", [4.0, 1.0, 0.0]);

    // Reihe 2: Positionen X: 1.0 bis 4.0 | Y: 2.0
    plant.create_bay("Bay-Tisch-5", [1.0, 2.0, 0.0]);
    plant.create_bay("Bay-Tisch-6", [2.0, 2.0, 0.0]);
    plant.create_bay("Bay-Tisch-7", [3.0, 2.0, 0.0]);
    plant.create_bay("Bay-Tisch-8", [4.0, 2.0, 0.0]);

    let plant_arc = Arc::new(RwLock::new(plant));

    // BasyxManager initialisieren
    let basyx_manager = Arc::new(BasyxManager::new(
        plant_arc.clone(),
        "http://localhost:8081/aasServer".to_string()
    ));

    // Automatische BaSyx-Registrierung im Hintergrund starten
    let basyx_init = basyx_manager.clone();
    tokio::spawn(async move {
        info!("📡 Versuche PlantModel bei BaSyx zu registrieren...");
        if let Err(e) = basyx_init.register_plant().await {
            error!("❌ BaSyx-Registrierung fehlgeschlagen: {}", e);
        }
    });

    // 1. Zuerst AgentManager (da SkillManager ihn braucht)
    let agent_manager = Arc::new(AgentManager::new(
        socket_manager.clone(), 
        resource_manager.clone(),
        plant_arc.clone()
    ));

    // 2. Dann SkillActionManager (nutzt jetzt die Variable agent_manager)
    let skill_manager = Arc::new(SkillActionManager::new(
        ros_client.node.clone(),
        plant_arc.clone(),
        agent_manager.clone(), // Jetzt existiert die Variable!
        resource_manager.clone()
    ));
    
    // RFID Manager initialisieren
    let rfid_manager = Arc::new(RfidManager::new(
        socket_manager.clone(),
        agent_manager.clone() // Er braucht Zugriff auf die Agents (Bays)
    ));

    // Initialisierung des AppState (SSoT)
    let shared_state = Arc::new(AppState {
        hardware_registry: RwLock::new(HashMap::new()),
        agent_manager,
        infra_config,
        data_config: Arc::new(DataConfigManager::new()),
        http_client: HttpClient::new(),
        socket_manager,
        inventory_manager,
        resource_manager: resource_manager.clone(), 
        skill_manager,
        path_manager: Arc::new(PathManager::new()),
        node_manager: Arc::new(NodeManager::new()),
        ros_client: ros_client.clone(),
        match_manager: Arc::new(RwLock::new(None)),
        world_state: RwLock::new(WorldState(HashSet::new())),
        planner: Arc::new(HTNPlanner {
            operators: HashMap::new(), 
            methods: HashMap::new(),
        }),
        rfid_manager,
        system_api,
        plant_model: plant_arc,
        basyx_manager,
    });

    // --- JuSt AnOtHeR lItTlE ROS 2 SETUP ---
    let _heartbeat = passform_agent_backend::ros::ros_heartbeat::Heartbeat::new(&ros_client.node, 1.0);

    // --- ROS 2 SUBSCRIBER SETUP ---
    let am_clone = shared_state.agent_manager.clone();
    let rt_handle = tokio::runtime::Handle::current();

    let _agent_sub = ros_client.node.create_subscription::<passform_agent_resources::msg::AgentInfo, _>(
        "agent_info",
        move |msg: passform_agent_resources::msg::AgentInfo| {
            let am = am_clone.clone();
            rt_handle.spawn(async move {
                am.sync_from_ros(
                    msg.agent_id,
                    msg.module_type,
                    msg.position.x as i32,
                    msg.position.y as i32,
                    msg.orientation as i32,
                    0, // status_code (0 = Status::Ok)
                ).await;
            });
        },
    ).expect("❌ Failed to create agent_info subscription");

    // --- watchdog ---
    let watchdog = Arc::new(Watchdog::new(shared_state.clone()));
    let watchdog_thread = watchdog.clone();
    tokio::spawn(async move {
        watchdog_thread.run().await;
    });

    // --- Protocol manager ---
    let mut protocol_manager = ProtocolManager::new(shared_state.clone());
    tokio::spawn(async move {
        protocol_manager.run().await;
    });

    // --- RFID manager ---
    let rfid_m_clone = shared_state.rfid_manager.clone();
    let rt_rfid = tokio::runtime::Handle::current();

    // --- Inventory manager ---
    let inv_m_clone = shared_state.inventory_manager.clone();
    let rt_inv = tokio::runtime::Handle::current();

   // In src/main.rs (Subscriber für detected_items)
    let _inventory_sub = ros_client.node.create_subscription::<passform_agent_resources::msg::WorldItem, _>(
        "detected_items",
        move |msg: passform_agent_resources::msg::WorldItem| {
            let im = inv_m_clone.clone();
            rt_inv.spawn(async move {
                // NUTZE DEN CONVERTER HIER:
                let new_item = passform_agent_backend::core::types::inventory::WorldItem {
                    name: msg.part.name,
                    uid: msg.uuid,
                    quantity: msg.quantity as i32,
                    // Hier war der Fehler:
                    location: passform_agent_backend::RosConverter::location_from_msg(&msg.location),
                };

                im.upsert_item(new_item).await;
            });
        },
    ).expect("❌ Failed to create inventory subscription");

    // Nutze einen Standard-Typ wie String, falls RfidScan noch nicht existiert
    let _rfid_sub = ros_client.node.create_subscription::<passform_agent_resources::msg::InfraRfidScan, _>(
    "rfid_updates",
    move |msg: passform_agent_resources::msg::InfraRfidScan| {
            let rm = rfid_m_clone.clone();
            rt_rfid.spawn(async move {
                let tag = if msg.is_detected { Some(msg.uid) } else { None };
                rm.handle_rfid_event(msg.bay_id, tag).await;
            });
        },
    ).expect("❌ Failed to create rfid_scan subscription");

    // --- 5. ROLLEN-WEICHE ---
    if role == SystemRole::Master {
        info!("👑 Modus: MASTER. Starte MatchManager...");
        let match_manager = Arc::new(MatchManager::new(shared_state.clone()));
        let mm_for_state = match_manager.clone();
        let state_lock = shared_state.clone();
        tokio::spawn(async move {
            let mut mm_slot = state_lock.match_manager.write().await;
            *mm_slot = Some(mm_for_state);
        });
        let mm_thread = match_manager.clone();
        tokio::spawn(async move {
            mm_thread.run().await;
        });
    }

    // --- 6. Axum Server Task ---
    let axum_state = shared_state.clone();
    let io_layer = layer.clone(); 
    let cors_layer = cors.clone(); // Klon für den Thread

    tokio::spawn(async move {
        let app = axum::Router::new()
            .route("/", axum::routing::get(root_handler))
            .route("/api/agents", axum::routing::get(get_agents))
            .route("/api/bays", axum::routing::get(get_bays))
            .route("/api/resources/specs", axum::routing::get(get_hardware_specs))
            .route("/api/resources/interfaces", axum::routing::get(get_ros_interfaces))
            .route("/api/planner/plan", axum::routing::post(handle_planning_request))
            .route("/basyx", axum::routing::any(BasyxManager::proxy_handler))
            .route("/basyx/", axum::routing::any(BasyxManager::proxy_handler))
            .route("/basyx/index.html", axum::routing::any(BasyxManager::proxy_handler))
            .route("/basyx/*path", axum::routing::any(BasyxManager::proxy_handler))
            .route("/aas-api/*path", axum::routing::any(BasyxManager::proxy_handler))
            .route("/api/inventory/basyx", axum::routing::get(get_inventory_basyx))
            .route("/api/inventory", axum::routing::get(get_inventory))
            .layer(cors_layer) // <--- Hier wird CORS jetzt korrekt gefunden
            .layer(io_layer)
            .with_state(axum_state);

        let addr = std::net::SocketAddr::from(([0, 0, 0, 0], 8080));
        let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
        info!("⚓ Axum API & Socket.io Server läuft auf http://{}", addr);
        axum::serve(listener, app).await.unwrap();
    });

    // 7. BT Test Logic
    let skill_lib = SkillLibrary::from_path(&resource_manager.skills_path);
    if let Some(complex_skill) = skill_lib.skills.iter().find(|s| s.skill_type == "CUSTOM") {
        let mut root_node = passform_agent_backend::behaviour_tree::build_tree(complex_skill, &skill_lib);
        tokio::spawn(async move {
            loop {
                let status = root_node.tick().await;
                if status != NodeStatus::Running { break; }
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            }
        });
    }

    // 8. Tauri App Run
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

async fn get_bays(AxumState(state): AxumState<Arc<AppState>>) -> Json<Vec<passform_agent_backend::Bay>> {
    let plant_guard = state.plant_model.read().await;
    // Wir geben einfach die Liste der Buchten aus dem PlantModel zurück
    Json(plant_guard.bays.clone())
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

async fn get_inventory(AxumState(state): AxumState<Arc<AppState>>) -> Json<Vec<serde_json::Value>> {
    let items_guard = state.inventory_manager.items.read().await;
    let list = items_guard
        .values()
        .map(|item| serde_json::to_value(item).unwrap_or_default())
        .collect();
    Json(list)
}

async fn get_inventory_basyx(AxumState(state): AxumState<Arc<AppState>>) -> Json<serde_json::Value> {
    // Ruft die neue Methode auf, die wir im InventoryManager eingebaut haben
    Json(state.inventory_manager.get_inventory_basyx_json().await)
}

async fn handle_planning_request(
    AxumState(state): AxumState<Arc<AppState>>,
    Json(payload): Json<serde_json::Value>
) -> Json<serde_json::Value> {
    let tasks: Vec<String> = payload["tasks"].as_array()
        .map(|t| t.iter().filter_map(|v| v.as_str().map(|s| s.to_string())).collect())
        .unwrap_or_default();
    let current_state = state.world_state.read().await.clone();
    match state.planner.plan(current_state, tasks) {
        Ok(plan) => Json(serde_json::json!({ "status": "success", "plan": plan })),
        Err(e) => Json(serde_json::json!({ "status": "error", "message": e })),
    }
}