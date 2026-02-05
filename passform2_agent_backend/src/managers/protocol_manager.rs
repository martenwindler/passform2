use std::sync::Arc;
use std::time::Duration;
use tracing::{info, error};
use rumqttc::{AsyncClient, MqttOptions, QoS, Event, Packet};

// Korrekte Pfade basierend auf deiner aktuellen Struktur
use crate::core::types::status::Status;
use crate::core::util::util::LogLevel;
use crate::AppState;

/// Befehle, die von den Protokollen an den Core gesendet werden
#[derive(Debug)]
pub enum InternalCommand {
    MoveTo { x: f64, y: f64, theta: f64 },
    Action { name: String, params: serde_json::Value },
    EmergencyStop,
}

pub struct ProtocolManager {
    state: Arc<AppState>,
    mqtt_client: Option<AsyncClient>, 
}

impl ProtocolManager {
    pub fn new(state: Arc<AppState>) -> Self {
        Self {
            state,
            mqtt_client: None,
        }
    }

    /// Haupt-Loop für den Protocol Manager
    pub async fn run(&mut self) {
        info!("🌐 Protocol Manager gestartet. Initialisiere Adapter...");

        // 1. VDA 5050 / MQTT Adapter starten
        if let Err(e) = self.setup_vda_mqtt().await {
            error!("❌ VDA 5050 MQTT Setup fehlgeschlagen: {}", e);
        }

        // 2. Deebot WLAN Brücke
        self.setup_deebot_bridge().await;
    }

    /// Konfiguration und Start der VDA 5050 MQTT Schnittstelle
    async fn setup_vda_mqtt(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let broker = "localhost"; 
        let port = 1883;

        let mut mqttoptions = MqttOptions::new("passform_core_vda", broker, port);
        mqttoptions.set_keep_alive(Duration::from_secs(5));

        // Fix E0282: Explizite Typ-Zuweisung für AsyncClient und EventLoop
        let (client, mut eventloop) = AsyncClient::new(mqttoptions, 10);
        self.mqtt_client = Some(client.clone());

        let order_topic = "uagv/v1/Agilex/RANGER-001/order";
        client.subscribe(order_topic, QoS::AtLeastOnce).await?;

        // Background Task für MQTT Events
        tokio::spawn(async move {
            info!("📡 VDA 5050 MQTT Loop aktiv.");
            loop {
                // poll() gibt ein Result<Event, ConnectionError> zurück
                match eventloop.poll().await {
                    Ok(notification) => {
                        if let Event::Incoming(Packet::Publish(p)) = notification {
                            Self::handle_vda_order(p.payload.to_vec()).await;
                        }
                    }
                    Err(e) => {
                        error!("MQTT Connection Error: {}", e);
                        tokio::time::sleep(Duration::from_secs(5)).await;
                    }
                }
            }
        });

        Ok(())
    }

    async fn handle_vda_order(payload: Vec<u8>) {
        if let Ok(json) = serde_json::from_slice::<serde_json::Value>(&payload) {
            info!("📥 VDA Order erhalten: {:?}", json);
            // Hier kommt später der Aufruf an den PathManager
        }
    }

    async fn setup_deebot_bridge(&self) {
        info!("🧹 Deebot WLAN-Adapter bereit (Wartet auf Python-Bridge)");
    }

    pub async fn broadcast_state(&self, topic: &str, payload: String) {
        if let Some(ref client) = self.mqtt_client {
            let _ = client.publish(topic, QoS::AtMostOnce, false, payload).await;
        }
    }
}