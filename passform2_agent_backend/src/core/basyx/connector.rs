use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;
use rclrs::Node;
use serde_json::Value;

use crate::basyx::rest_server::RestServer;
use crate::log_level::LogLevel;
use passform_msgs::srv::Discover;

/// Der Connector verwaltet die aktive Verbindung zum BaSyx-Infrastruktur-Server.
pub struct RestServerConnector {
    node: Option<Arc<Node>>,
    client: Option<RestServer>,
    managed_aas: HashMap<String, String>, // ID -> Type (als String)
    timeout: Duration,
}

impl RestServerConnector {
    pub fn new(node: Option<Arc<Node>>, timeout_secs: f64) -> Self {
        Self {
            node,
            client: None,
            managed_aas: HashMap::new(),
            timeout: Duration::from_secs_f64(timeout_secs),
        }
    }

    /// Verbindet basierend auf der Discovery-Response
    pub fn connect_with_discover(&mut self, resp: &Discover::Response) {
        self.log(&format!("Verbindungsinfos erhalten: {:?}", resp), LogLevel::Debug);
        
        for ds in &resp.server_description {
            if ds.name.to_lowercase() == "basyx" {
                self.log("BaSyx Infrastruktur gefunden.", LogLevel::Debug);
                
                let data: HashMap<String, String> = ds.values.iter()
                    .map(|kv| (kv.key.clone(), kv.value.clone()))
                    .collect();

                if data.get("registry_type").map(|s| s.as_str()) == Some("REST") {
                    if let Some(host) = data.get("registry_host") {
                        self.connect(host);
                        return;
                    }
                }
            }
        }
        self.log("Keine BaSyx-Informationen in Discovery gefunden.", LogLevel::Warn);
    }

    pub fn connect(&mut self, registry_host: &str) {
        self.log(&format!("Erstelle BaSyx-Client für Registry: {}", registry_host), LogLevel::Info);
        // Wir nutzen hier den RestServer, den wir bereits portiert haben
        self.client = Some(RestServer::new(registry_host, 8081, 8082, self.timeout.as_secs() as u32));
    }

    /// Generische Add-Methode (ersetzt das Python isinstance-Gefriemel)
    pub fn add_aas(&mut self, aas_json: Value) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(ref client) = self.client {
            let id = aas_json["identification"]["id"].as_str()
                .ok_or("AAS hat keine ID")?.to_string();
            
            client.add_aas(aas_json)?;
            self.managed_aas.insert(id, "AAS".to_string());
            Ok(())
        } else {
            Err("Nicht verbunden".into())
        }
    }

    pub fn add_submodel(&mut self, aas_id: &str, submodel_json: Value) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(ref client) = self.client {
            client.add_submodel(aas_id, submodel_json)?;
            Ok(())
        } else {
            Err("Nicht verbunden".into())
        }
    }

    /// Cleanup: Löscht alle registrierten AAS vom Server (Drop-Trait Ersatz für __del__)
    pub fn destroy(&mut self) {
        if let Some(ref client) = self.client {
            for (id, _) in &self.managed_aas {
                self.log(&format!("Entferne AAS {} vom Server...", id), LogLevel::Info);
                let _ = client.remove_aas(id);
            }
        }
    }

    /// Zentrales Logging, das ROS-Logger oder Print nutzt
    pub fn log(&self, msg: &str, lvl: LogLevel) {
        if let Some(ref node) = self.node {
            let logger = node.get_logger();
            match lvl {
                LogLevel::Debug => logger.debug(msg),
                LogLevel::Info => logger.info(msg),
                LogLevel::Warn => logger.warn(msg),
                LogLevel::Error => logger.error(msg),
                LogLevel::Fatal => logger.fatal(msg),
            }
        } else {
            println!("[{:?}] {}", lvl, msg);
        }
    }
}

/// Factory Ersatz durch Rust Enum/Match Pattern
pub enum ConnectorConfig<'a> {
    Discover(&'a Discover::Response),
    Params(&'a HashMap<String, String>),
    Host(&'a str),
}

pub fn connector_factory(config: ConnectorConfig, node: Option<Arc<Node>>) -> RestServerConnector {
    let mut connector = RestServerConnector::new(node, 3.0);
    match config {
        ConnectorConfig::Discover(resp) => connector.connect_with_discover(resp),
        ConnectorConfig::Params(map) => {
            if let Some(host) = map.get("registry_host") {
                connector.connect(host);
            }
        },
        ConnectorConfig::Host(host) => connector.connect(host),
    }
    connector
}

// Implementierung für automatisches Cleanup beim Beenden
impl Drop for RestServerConnector {
    fn drop(&mut self) {
        self.destroy();
    }
}