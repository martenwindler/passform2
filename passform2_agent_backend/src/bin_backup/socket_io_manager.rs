use socketioxide::{
    extract::{Data, SocketRef},
    SocketIo,
};
use std::sync::Arc;
use serde_json::json;
use tracing::{info, error, warn};
use tokio::sync::RwLock;

// Wir bauen eine Struktur, die das IO-Handle hält und die Logik bündelt
pub struct SocketManager {
    io: SocketIo,
    state: Arc<crate::AppState>,
}

impl SocketManager {
    /// Konstruktor: Hier konfigurieren wir die "Engine" (Ersatz für die Python-Initialisierung)
    pub fn init(state: Arc<crate::AppState>) -> (socketioxide::layer::SocketIoLayer, Arc<Self>) {
        let (layer, io) = SocketIo::builder()
            .ping_interval(std::time::Duration::from_secs(10))
            .ping_timeout(std::time::Duration::from_secs(30))
            // In Rust ist "check_cors" Teil der Axum/Tower-Middleware (siehe main.rs)
            .build_layer();

        let manager = Arc::new(Self {
            io: io.clone(),
            state,
        });

        // Events registrieren
        manager.register_events();

        (layer, manager)
    }

    /// Registriert alle Event-Handler (Ersatz für _register_events)
    fn register_events(self: &Arc<Self>) {
        let manager = Arc::clone(self);
        
        self.io.ns("/", move |socket: SocketRef| {
            let m = Arc::clone(&manager);
            
            // --- CONNECT EVENT ---
            info!("🔗 Client verbunden: {} | Gesamt-Clients: {}", 
                socket.id, m.io.num_clients());

            // Sofortiger Status-Sync an den EINEN Client
            socket.emit("socket_status", true).ok();

            // Initial-Sync Task starten (Ersatz für _handle_new_connection)
            let s_ref = socket.clone();
            let m_sync = Arc::clone(&m);
            tokio::spawn(async move {
                m_sync.sync_client_state(s_ref).await;
            });

            // --- DISCONNECT EVENT ---
            socket.on_disconnect(move |s: SocketRef| {
                info!("❌ Client getrennt: {}", s.id);
                // Globaler Status-Broadcast (Rote Lampe triggern)
                s.broadcast().emit("socket_status", false).ok();
            });

            // --- HARDWARE UPDATES (Pi -> Backend -> Browser) ---
            socket.on("pi_hardware_update", move |s: SocketRef, Data::<serde_json::Value>(data)| {
                info!("🖥️ Hardware-Update empfangen");
                // Broadcast an alle anderen (Browser-Tabs)
                s.broadcast().emit("hardware_update", data).ok();
            });

            // --- CONFIG PUSH (Browser -> Backend -> SSoT) ---
            socket.on("push_config", move |s: SocketRef, Data::<serde_json::Value>(data)| {
                let m_config = Arc::clone(&m);
                tokio::spawn(async move {
                    if let Some(agents) = data.get("agents") {
                        info!("💾 SSoT Update via Socket empfangen");
                        // Hier würde der Aufruf an den ConfigManager erfolgen
                        s.broadcast().emit("active_agents", json!({"agents": agents})).ok();
                    }
                });
            });
        });
    }

    /// Der "Ultimate Sync" (Ersatz für handle_new_connection)
    async fn sync_client_state(&self, socket: SocketRef) {
        let s = &self.state;

        // 1. Modus senden
        let mode = s.config.get_current_mode().await;
        socket.emit("mode", mode.to_string()).ok();

        // 2. Agenten senden (Echtzeit-Stand)
        let timeout = s.config.settings.read().await.timeout_factor;
        let hz = s.config.settings.read().await.heartbeat_hz;
        let agents = s.agent_manager.get_active_agents_json(1.0/hz * timeout).await;
        socket.emit("active_agents", agents).ok();

        // 3. System-Logs nachliefern
        let logs = s.agent_manager.logs.read().await;
        socket.emit("system_logs_init", &*logs).ok();

        info!("✅ Sync für Client {} abgeschlossen.", socket.id);
    }

    /// Ersatz für emit_event_sync (Thread-sicheres Senden von überall)
    pub fn broadcast(&self, event: &str, data: serde_json::Value) {
        if let Err(e) = self.io.emit(event, data) {
            error!("❌ Kritischer Sende-Fehler bei '{}': {}", event, e);
        }
    }
}