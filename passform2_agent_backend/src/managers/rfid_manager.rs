use std::sync::Arc;
use std::collections::HashMap;
use std::time::Instant;
use tokio::sync::RwLock;

use crate::core::types::bay_aas::Bay;
use crate::core::types::Status;
use crate::managers::agent_manager::AgentManager;
use crate::managers::socket_io_manager::SocketIoManager;

use tracing::{info, error};

/// RfidManager: Zentralisiert die Hardware-Events der Bay-Kartenleser.
/// Verwalte den Zustand der physischen Buchten (Bays) und informiert das System.
pub struct RfidManager {
    // Speichert den Zustand aller Bays (Buchten) thread-sicher
    pub bays: RwLock<HashMap<String, Bay>>, 
    pub socket_manager: Arc<SocketIoManager>,
    pub agent_manager: Arc<AgentManager>, // Ermöglicht Interaktion mit anderen Agenten
}

impl RfidManager {
    pub fn new(socket_manager: Arc<SocketIoManager>, agent_manager: Arc<AgentManager>) -> Self {
        Self {
            bays: RwLock::new(HashMap::new()),
            socket_manager,
            agent_manager,
        }
    }

    /// Verarbeitet ein RFID-Event eines Agenten vom Typ "Bay"
    /// tag = Some(uid) -> Modul angedockt
    /// tag = None      -> Modul entfernt
    pub async fn handle_rfid_event(&self, bay_id: String, rfid_tag: Option<String>) {
        let mut bays_guard = self.bays.write().await;
        
        // 1. Bay finden oder mit Standardwerten initialisieren
        // Nutzt den 3-Argument-Konstruktor des neuen Bay-Typs
        let bay = bays_guard.entry(bay_id.clone()).or_insert_with(|| {
            // Das vierte Argument 'false' gibt an, dass es eine physische Bucht ist
            Bay::new(&bay_id, "Unbekannte Bay", [0.0, 0.0, 0.0], false)
        });

        // 2. Watchdog füttern: Zeitstempel aktualisieren & Timeout-Flag zurücksetzen
        bay.last_update = Instant::now();
        bay.has_timeout = false;

        // 3. Status-Logik (Zentralisiert statt im Core-Type)
        match rfid_tag {
            Some(tag) => {
                info!("🏷️ RFID Scan an Bay {}: Modul {}", bay_id, tag);
                bay.current_rfid = Some(tag.clone());
                bay.module_uuid = tag;
                bay.occupation = true;
                bay.status = Status::Running; // Bay ist im aktiven Betrieb
            },
            None => {
                info!("🏷️ RFID Release an Bay {}", bay_id);
                bay.current_rfid = None;
                bay.module_uuid = String::new();
                bay.occupation = false;
                bay.status = Status::Ok; // Bay ist frei und bereit
            }
        }
        
        // 4. UI/Frontend via Socket.io benachrichtigen
        // Wir lassen das Guard hier kurz los, indem wir die Liste klonen
        drop(bays_guard); 
        self.broadcast_bay_states().await;
    }

    /// Sendet den aktuellen Zustand aller Buchten an verbundene Clients
    async fn broadcast_bay_states(&self) {
        let bays_list: Vec<Bay> = {
            let guard = self.bays.read().await;
            guard.values().cloned().collect()
        };
        
        let _ = self.socket_manager.emit_event(
            "bay_update",
            serde_json::json!({ "bays": bays_list })
        ).await;
    }
}