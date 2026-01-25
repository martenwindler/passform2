use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::collections::HashMap;

use rclrs;
use rclrs::{Node, Publisher, Subscription, Client, QOS_PROFILE_DEFAULT};
use rclrs_lifecycle::{LifecycleNode, LifecycleState, TransitionResult};

// Nachrichten & Services
use passform_msgs::msg::{Inventory, ModuleStatus, Item};
use passform_msgs::srv::{RegisterModule, RegisterModuleRequest};
use nav2_msgs::srv::{ManageLifecycleNodes, ManageLifecycleNodesRequest};

// Eigene Module
use crate::registration::DiscoverPassform;
use crate::basyx::rest_server::RestServer;
use crate::network;

pub struct ModuleManager {
    node: Arc<Node>,
    // State
    basyx: Arc<Mutex<Option<RestServer>>>,
    base_services: HashMap<String, String>,
    inventory: Arc<Mutex<Vec<Item>>>,
    
    // Interfaces
    registration_client: Option<Arc<Client<RegisterModule>>>,
    status_publisher: Option<Arc<Publisher<ModuleStatus>>>,
    inventory_sub: Option<Arc<Subscription<Inventory>>>,
    status_timer: Option<Arc<rclrs::Timer>>,

    // Parameter Cache
    uuid: String,
    lc_manager_names: Vec<String>,
}

impl ModuleManager {
    pub fn new(node_name: &str) -> Self {
        let node = rclrs::Node::new(node_name).unwrap();
        
        // Parameter deklarieren
        let uuid = node.declare_parameter("uuid").get_string();
        let lc_manager_names = node.declare_parameter("lc_manager_names").get_string_array();

        Self {
            node,
            basyx: Arc::new(Mutex::new(None)),
            base_services: HashMap::new(),
            inventory: Arc::new(Mutex::new(Vec::new())),
            registration_client: None,
            status_publisher: None,
            inventory_sub: None,
            status_timer: None,
            uuid,
            lc_manager_names,
        }
    }

    /// Hilfsmethode zur Transition untergeordneter LC-Knoten (Skill Manager etc.)
    fn manage_lifecycle_nodes(&self, transition: i32) -> Result<(), Box<dyn std::error::Error>> {
        for lc_name in &self.lc_manager_names {
            let service_name = format!("{}/{}/manage_nodes", self.node.get_namespace(), lc_name)
                .replace("//", "/");
            
            let client = self.node.create_client::<ManageLifecycleNodes>(&service_name)?;
            
            if !client.wait_for_service(Duration::from_secs(2))? {
                return Err(format!("LC Manager {} nicht erreichbar", service_name).into());
            }

            let request = ManageLifecycleNodesRequest { command: transition as u8 };
            let future = client.call_async(&request);
            
            // Warten auf Ergebnis (Blocking wait für Transition)
            let result = future.read_value()?; 
            if result.is_none() || !result.unwrap().success {
                return Err(format!("Transition für {} fehlgeschlagen", lc_name).into());
            }
        }
        Ok(())
    }

    /// Registrierungs-Request an den Master-Node
    fn perform_registration(&self, reg_type: i32) -> Result<(), Box<dyn std::error::Error>> {
        let client = self.registration_client.as_ref()
            .ok_or("Registrierungs-Client nicht initialisiert")?;

        let mut request = RegisterModuleRequest::default();
        request.reg_type = reg_type as u8;
        request.module.uuid = self.uuid.clone();
        
        // IP-Logik aus unserem network.rs
        request.id = network::get_ip().to_string();
        request.id_type = "IP".to_string();

        let future = client.call_async(&request);
        let result = future.read_value()?;
        
        if result.is_none() || !result.unwrap().success {
            return Err("Registrierung am Master fehlgeschlagen".into());
        }
        Ok(())
    }
}

impl LifecycleNode for ModuleManager {
    fn on_configure(&mut self, _state: &LifecycleState) -> TransitionResult {
        // 1. Discovery
        let mut discoverer = DiscoverPassform::new(Arc::clone(&self.node));
        if let Err(_) = discoverer.discover() { return TransitionResult::Failure; }
        
        self.base_services = discoverer.get_services().clone();
        
        // 2. BaSyx Initialisierung
        if let Ok(client) = discoverer.start_basyx() {
            // In Rust erstellen wir hier das AAS direkt über den Client
            // client.add_aas(...); 
            *self.basyx.lock().unwrap() = Some(client);
        }

        // 3. Interfaces erstellen
        let reg_topic = self.base_services.get("registration").unwrap();
        self.registration_client = Some(self.node.create_client::<RegisterModule>(reg_topic).unwrap());

        self.status_publisher = Some(self.node.create_publisher::<ModuleStatus>(
            "/module/status", QOS_PROFILE_DEFAULT).unwrap());

        TransitionResult::Success
    }

    fn on_activate(&mut self, _state: &LifecycleState) -> TransitionResult {
        // 1. Am Master registrieren
        if let Err(e) = self.perform_registration(1) { // 1 = REGISTER
            eprintln!("Aktivierung fehlgeschlagen: {}", e);
            return TransitionResult::Failure;
        }

        // 2. Untergeordnete LC-Nodes starten (STARTUP = 1)
        if let Err(_) = self.manage_lifecycle_nodes(1) { return TransitionResult::Failure; }

        TransitionResult::Success
    }

    fn on_deactivate(&mut self, _state: &LifecycleState) -> TransitionResult {
        // 1. Untergeordnete LC-Nodes stoppen (RESET = 2)
        let _ = self.manage_lifecycle_nodes(2);
        
        // 2. Vom Master abmelden (2 = UNREGISTER)
        let _ = self.perform_registration(2);

        TransitionResult::Success
    }
}