#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();
    info!("Starter Manager...");
    // Hier deine Logik aufrufen
    Ok(())
}