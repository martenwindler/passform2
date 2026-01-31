use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::fs::File;

use rclrs;
use rclrs::{Node, Publisher, Service, Subscription};
use rclrs_lifecycle::{LifecycleNode, LifecycleState, TransitionResult};

// Nachrichten & Services
use passform_msgs::msg::{Inventory, Location, AreaOfInterest};
use passform_msgs::srv::{UpdateInventory, UpdateInventoryRequest, UpdateInventoryResponse};

// Eigene Module
use crate::registration::DiscoverPassform;
use crate::core::basyx::rest_server::RestServer;
use crate::helper::load_yaml; // Unsere neue YAML-Funktion

/// Rust-Entsprechung für ModuleInventory
pub struct ModuleInventory {
    // Hier käme deine Lagerlogik rein (Maps von UIDs zu Items etc.)
}

pub struct InventoryManager {
    node: Arc<Node>,
    inventory: Arc<Mutex<ModuleInventory>>,
    basyx: Option<RestServer>,
    
    // Interfaces
    inventory_publisher: Option<Arc<Publisher<Inventory>>>,
    add_service: Option<Arc<Service<UpdateInventory>>>,
    use_service: Option<Arc<Service<UpdateInventory>>>,
    get_service: Option<Arc<Service<UpdateInventory>>>,
    status_timer: Option<Arc<rclrs::Timer>>,
    
    // Parameter
    uuid: String,
    storage_file: String,
}

impl InventoryManager {
    pub fn new(node_name: &str) -> Self {
        let node = rclrs::Node::new(node_name).unwrap();
        
        // Parameter deklarieren
        let uuid = node.declare_parameter("uuid").get_string();
        let storage_file = node.declare_parameter("storage_file").get_string();

        Self {
            node,
            inventory: Arc::new(Mutex::new(ModuleInventory::new())),
            basyx: None,
            inventory_publisher: None,
            add_service: None,
            use_service: None,
            get_service: None,
            status_timer: None,
            uuid,
            storage_file,
        }
    }

    fn publish_status(&self) {
        if let Some(ref publ) = self.inventory_publisher {
            let inv_guard = self.inventory.lock().unwrap();
            let msg = inv_guard.to_ros_msg(); // Konvertierungsmethode
            publ.publish(&msg).unwrap();
        }
    }

    fn create_interfaces(&mut self) {
        let node_name = self.node.get_name();
        
        // Add Service
        let inv_add = Arc::clone(&self.inventory);
        self.add_service = Some(self.node.create_service::<UpdateInventory, _>(
            &format!("{}/add", node_name),
            move |_header, req| {
                let mut inv = inv_add.lock().unwrap();
                // Logik: inv.add(req.item) ...
                UpdateInventoryResponse { success: true, ..Default::default() }
            },
        ).unwrap());

        // Publisher
        self.inventory_publisher = Some(self.node.create_publisher::<Inventory>(
            "inventory",
            rclrs::QOS_PROFILE_DEFAULT,
        ).unwrap());

        // Status Timer (1Hz)
        let node_handle = Arc::clone(&self.node);
        // In Rust-rclrs werden Timer oft über den Node-Loop verwaltet
    }
}

impl LifecycleNode for InventoryManager {
    fn on_configure(&mut self, _state: &LifecycleState) -> TransitionResult {
        println!("Configuring InventoryManager...");

        // 1. Discovery & BaSyx
        let mut discoverer = DiscoverPassform::new(Arc::clone(&self.node));
        if let Err(e) = discoverer.discover() {
            eprintln!("Discovery failed: {}", e);
            return TransitionResult::Failure;
        }

        match discoverer.start_basyx() {
            Ok(client) => self.basyx = Some(client),
            Err(e) => {
                eprintln!("BaSyx start failed: {}", e);
                return TransitionResult::Failure;
            }
        }

        // 2. Storage Locations laden
        if let Err(e) = self.add_storage_locations() {
            println!("Warning: Storage file loading failed: {}. Using defaults.", e);
            // Default-Logik hier...
        }

        // 3. Interfaces
        self.create_interfaces();

        TransitionResult::Success
    }

    fn on_cleanup(&mut self, _state: &LifecycleState) -> TransitionResult {
        self.add_service = None;
        self.use_service = None;
        self.inventory_publisher = None;
        TransitionResult::Success
    }
}