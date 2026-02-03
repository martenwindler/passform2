use socketioxide::SocketIo;
use serde_json::Value;
use tracing::{info, error};

pub struct SocketIoManager {
    pub io: SocketIo,
}

impl SocketIoManager {
    pub fn new(io: SocketIo) -> Self {
        Self { io }
    }

    /// Generischer Emitter (Ersatz für emit_event_sync aus Python)
    /// In Rust nutzen wir einfach tokio::spawn, wenn wir aus einem 
    /// synchronen Kontext kommen, ansonsten direkt .await
    pub async fn emit_event(&self, event: impl Into<String>, data: Value) {
        let event_name = event.into();
        // Das .emit() von socketioxide ist intern bereits hochoptimiert
        if let Err(e) = self.io.emit(event_name.clone(), data) {
            error!("❌ Fehler beim Senden von '{}': {:?}", event_name, e);
        }
    }

    /// Spezifischer Broadcast für Agenten (Move-Sync)
    pub async fn broadcast_active_agents(&self, agents: Value) {
        let _ = self.io.emit("active_agents", agents);
    }

    pub async fn broadcast_hardware_update(&self, data: Value) {
        info!("📢 Broadcasting Hardware-Update via Socket.io");
        let _ = self.io.emit("hardware_update", data);
    }
}