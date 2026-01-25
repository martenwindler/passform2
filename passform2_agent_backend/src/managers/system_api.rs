use tracing::info;

pub struct SystemApi {
    start_time: std::time::Instant,
}

impl SystemApi {
    pub fn new() -> Self {
        info!("🖥️ System-API initialisiert.");
        Self {
            start_time: std::time::Instant::now(),
        }
    }

    pub async fn get_uptime(&self) -> u64 {
        self.start_time.elapsed().as_secs()
    }

    pub async fn check_health(&self) -> bool {
        true
    }
}

impl Default for SystemApi {
    fn default() -> Self {
        Self::new()
    }
}