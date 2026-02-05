use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{info, warn, error};
use serde_json::{json, Value}; // Value hinzugefügt

use crate::core::types::inventory::WorldItem;
use crate::managers::socket_io_manager::SocketIoManager;
use crate::managers::basyx_manager::BasyxManager; // Import für den Kreis-Schluss

pub struct InventoryManager {
    pub items: RwLock<HashMap<String, WorldItem>>,
    pub socket_manager: Arc<SocketIoManager>,
}

impl InventoryManager {
    pub fn new(socket_manager: Arc<SocketIoManager>) -> Self {
        Self {
            items: RwLock::new(HashMap::new()),
            socket_manager,
        }
    }

    /// Gibt das gesamte Inventar im BaSyx-Format zurück (Der Kreis-Schluss)
    pub async fn get_inventory_basyx_json(&self) -> Value {
        let items_guard = self.items.read().await;
        let basyx_items: Vec<Value> = items_guard
            .values()
            .map(|item| BasyxManager::render_item(item)) // Nutzt den zentralen Renderer!
            .collect();

        json!({
            "idShort": "InventoryList",
            "modelType": "SubmodelElementCollection",
            "value": basyx_items
        })
    }

    pub async fn upsert_item(&self, item: WorldItem) {
        let mut items = self.items.write().await;
        info!("📦 Inventory: Update für Item {} (Menge: {})", item.uid, item.quantity);
        items.insert(item.uid.clone(), item);
        
        drop(items);
        self.broadcast_inventory().await;
    }

    pub async fn merge_items(&self, source_uid: &str, target_uid: &str) -> Result<(), String> {
        let mut items = self.items.write().await;

        if !items.contains_key(source_uid) || !items.contains_key(target_uid) {
            return Err("Eines der Items wurde im Inventar nicht gefunden.".to_string());
        }

        let source_qty = items.get(source_uid).unwrap().quantity;
        
        if let Some(target_item) = items.get_mut(target_uid) {
            target_item.quantity += source_qty;
            info!("➕ Merge: {} Stück von {} nach {} transferiert.", source_qty, source_uid, target_uid);
        }

        items.remove(source_uid);
        drop(items);
        self.broadcast_inventory().await;
        Ok(())
    }

    pub async fn split_item(&self, uid: &str, amount: i32) -> Result<WorldItem, String> {
        let mut items = self.items.write().await;
        let item = items.get_mut(uid).ok_or_else(|| "Item nicht gefunden".to_string())?;

        if amount > item.quantity {
            return Err(format!("Unzureichende Menge: {} vorhanden, {} angefordert", item.quantity, amount));
        }

        item.quantity -= amount;
        let mut new_split_item = item.clone();
        new_split_item.quantity = amount;
        
        if item.quantity == 0 {
            items.remove(uid);
        }

        drop(items);
        self.broadcast_inventory().await;
        Ok(new_split_item)
    }

    pub async fn broadcast_inventory(&self) {
        let items_guard = self.items.read().await;
        let inventory_list: Vec<WorldItem> = items_guard.values().cloned().collect();
        
        self.socket_manager.emit_event(
            "inventory_update",
            json!({ "items": inventory_list })
        ).await;
    }

    pub async fn remove_item(&self, uid: &str) {
        let mut items = self.items.write().await;
        if items.remove(uid).is_some() {
            info!("🗑️ Item {} aus Inventar entfernt.", uid);
            drop(items);
            self.broadcast_inventory().await;
        }
    }
}