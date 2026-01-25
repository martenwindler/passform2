use std::sync::Arc;
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};
use serde_json::json;
use tokio::sync::RwLock;
use tracing::{info, error, warn};
use rppal::spi::{Spi, Bus, SlaveSelect, Mode};
use rppal::gpio::Gpio;

// --- DOMAIN MODELS ---

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum HardwareStatus {
    Online,
    Missing,
    Error,
}

#[derive(Serialize, Deserialize)]
struct PositionData {
    x: i32,
    y: i32,
}

// --- MANAGER ---

pub struct NfcManager {
    status: Arc<RwLock<HardwareStatus>>,
    running: Arc<RwLock<bool>>,
    // In Rust bruukt wi mfrc522 crate tosamen mit rppal
    // För dit Bispeel nimm an, wi hebbt en lütten Wrapper
}

impl NfcManager {
    pub fn new() -> Self {
        let status = Arc::new(RwLock::new(HardwareStatus::Missing));
        
        // Check Hardware (rppal prüft dat bi'n Start)
        let mut initial_status = HardwareStatus::Missing;
        match Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0) {
            Ok(_) => {
                info!("✅ RFID-RC522 (SPI) erfolgreich initialisiert.");
                initial_status = HardwareStatus::Online;
            },
            Err(e) => {
                warn!("⚠️ NFC-Hardware nich funnen: {}", e);
            }
        }

        Self {
            status: Arc::new(RwLock::new(initial_status)),
            running: Arc::new(RwLock::new(false)),
        }
    }

    pub async fn get_status(&self) -> HardwareStatus {
        self.status.read().await.clone()
    }

    // --- SCHREIB-LOGIK (Ersatz för write_position) ---

    pub async fn write_position(&self, x: i32, y: i32, timeout_secs: u64) -> String {
        let data = json!({ "x": x, "y": y }).to_string();
        self.write_tag(data, timeout_secs).await
    }

    pub async fn write_tag(&self, data: String, timeout_secs: u64) -> String {
        if self.get_status().await != HardwareStatus::Online {
            return "no_hardware".to_string();
        }

        info!("💾 Warte op Chip to'n Schrieven: {}", data);
        let start_time = Instant::now();
        let timeout = Duration::from_secs(timeout_secs);

        while start_time.elapsed() < timeout {
            // HIER kummt de echte SPI-Schreibbefehl rin
            // In Rust mookt wi dat dörch den mfrc522 Crate
            let success = true; // Placeholder för de Hardware-Aktion

            if success {
                info!("✅ Schrieven erfolgreich.");
                return "success".to_string();
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
        }

        "timeout".to_string()
    }

    // --- LESE-LOGIK (Ersatz för read_position) ---

    pub async fn read_position(&self, timeout_secs: u64) -> Option<(i32, i32)> {
        if self.get_status().await != HardwareStatus::Online {
            return None;
        }

        let start_time = Instant::now();
        let timeout = Duration::from_secs(timeout_secs);

        while start_time.elapsed() < timeout {
            // Placeholder för SPI-Read
            let raw_text = "{\"x\": 5, \"y\": 10}"; 
            if let Ok(pos) = serde_json::from_str::<PositionData>(raw_text) {
                return Some((pos.x, pos.y));
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
        None
    }

    // --- BACKGROUND READING (Ersatz för _read_loop) ---

    pub async fn start_reading(&self, io: socketioxide::SocketIo) {
        let status = self.status.clone();
        let running = self.running.clone();

        if *status.read().await != HardwareStatus::Online { return; }
        
        let mut is_running = running.write().await;
        if *is_running { return; }
        *is_running = true;

        info!("📡 NFC Hintergrund-Task startet.");

        tokio::spawn(async move {
            loop {
                // In Rust mookt wi dat ahn circular imports, 
                // indem wi de SocketIo Instanz dörchgeven.
                
                // Placeholder för Hardware Scan
                let card_id = "123-456-789"; 
                
                info!("🏷️ RFID Scan: {}", card_id);
                io.emit("rfid_found", card_id).ok();

                tokio::time::sleep(Duration::from_secs(2)).await;
                
                if !*running.read().await { break; }
            }
        });
    }
}