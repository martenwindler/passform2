use std::collections::HashMap;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use rclrs;
use rclrs::Node;
use passform_msgs::srv::Discover;

// Importiere deinen Rust-BaSyx-Client
use crate::basyx::rest_server::RestServer;

pub struct DiscoverPassform {
    node: Arc<Node>,
    discovery_performed: bool,
    base_services: HashMap<String, String>,
    basyx_config: HashMap<String, String>,
}

impl DiscoverPassform {
    pub fn new(node: Arc<Node>) -> Self {
        Self {
            node,
            discovery_performed: false,
            base_services: HashMap::new(),
            basyx_config: HashMap::new(),
        }
    }

    /// Führt die Discovery aus (Synchroner Call im Rust-Style)
    pub fn discover(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let discover_topic = "/passform/discover";
        let client = self.node.create_client::<Discover>(discover_topic)?;

        // Warten auf den Service (ähnlich wie wait_for_service)
        while !client.wait_for_service(Duration::from_secs(1))? {
            if !rclrs::ok() {
                return Err("Interrupted while waiting for discovery service".into());
            }
            println!("Warte auf Discovery Service '{}'...", discover_topic);
        }

        let request = Discover::Request::default();
        
        // In Rust nutzen wir meistens einen synchronen Call-Ansatz für Discovery beim Startup
        let future = client.call_async(&request);
        
        // Polling der Future (Ersatz für das Python spin_once Konstrukt)
        while rclpy::ok() {
            if let Some(response) = future.read_value()? {
                self.parse_response(response)?;
                break;
            }
            // Verhindert CPU-Maximierung beim Warten
            thread::sleep(Duration::from_millis(100));
        }

        self.discovery_performed = true;
        Ok(())
    }

    fn parse_response(&mut self, resp: Discover::Response) -> Result<(), Box<dyn std::error::Error>> {
        if resp.server_description.is_empty() {
            return Err("Discovery erhalten, aber server_description ist leer.".into());
        }

        for ds in resp.server_description {
            let data: HashMap<String, String> = ds.values
                .into_iter()
                .map(|kv| (kv.key, kv.value))
                .collect();

            match ds.name.to_lowercase().as_str() {
                "services" => {
                    self.base_services = data;
                },
                "basyx" => {
                    self.basyx_config = data;
                },
                _ => {}
            }
        }

        Ok(())
    }

    /// Startet den BaSyx-Server basierend auf den Discovery-Daten
    pub fn start_basyx(&self) -> Result<RestServer, Box<dyn std::error::Error>> {
        if !self.discovery_performed {
            return Err("Discovery wurde noch nicht ausgeführt!".into());
        }

        let reg_type = self.basyx_config.get("registry_type")
            .ok_or("BaSyx registry_type fehlt")?;

        if reg_type.to_lowercase() != "rest" {
            return Err(format!("Nur REST BaSyx unterstützt, Typ ist aber: {}", reg_type).into());
        }

        let host = self.basyx_config.get("registry_host")
            .ok_or("BaSyx registry_host fehlt")?;

        // Erstellt den Rust-Client (Standardports 8081/8082)
        Ok(RestServer::new(host, 8081, 8082, 3))
    }

    pub fn get_services(&self) -> &HashMap<String, String> {
        &self.base_services
    }
}