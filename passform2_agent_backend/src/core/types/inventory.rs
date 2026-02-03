use serde::{Serialize, Deserialize};
use serde_json::{json, Value};
use crate::core::util::helper::sanitize_id;

// Fix E0599: Trait ToBasyx muss importiert werden, damit die Methode sichtbar ist
use crate::core::types::location::{WorldLocation, ToBasyx}; 

// Wir nutzen den Alias 'passform_agent_resources' aus deiner Cargo.toml
use passform_agent_resources::msg::{WorldItem as ItemMsg, WorldPart as PartMsg};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct WorldItem {
    pub name: String,
    pub uid: String,
    pub quantity: i32,
    pub location: WorldLocation,
}

impl WorldItem {
    pub fn new(name: &str, uid: &str, quantity: i32, location: Option<WorldLocation>) -> Self {
        assert!(!uid.is_empty(), "unique id must not be empty");
        Self {
            name: name.to_string(),
            uid: uid.to_string(),
            quantity: if quantity < 0 { 0 } else { quantity },
            location: location.unwrap_or_default(),
        }
    }

    pub fn add(&mut self, other: &mut WorldItem) -> Result<(), String> {
        if self.uid != other.uid {
            return Err(format!(
                "Adding quantity to not matching item IDs: {} to {}",
                other.uid, self.uid
            ));
        }
        self.quantity += other.quantity;
        other.quantity = 0; 
        Ok(())
    }

    pub fn use_qty(&mut self, mut quantity: i32) -> WorldItem {
        if quantity < 0 { quantity = 0; }
        if quantity > self.quantity {
            quantity = 0;
        }
        
        let mut new_item = self.clone();
        new_item.quantity = quantity;
        self.quantity -= quantity;
        new_item
    }

    pub fn has_location(&self) -> bool {
        !self.location.get_aoi_uid().is_empty()
    }

    /// Konvertiert in eine ROS-Nachricht
    pub fn to_msg(&self) -> ItemMsg {
        ItemMsg {
            part: PartMsg {
                uuid: self.uid.clone(),
                name: self.name.clone(),
                grasp_options: Vec::new(), 
                // Fix E0308: Inertia ist eine Struktur, kein Vec. 
                // .default() erzeugt eine gültige, leere Inertia-Nachricht.
                inertia: geometry_msgs::msg::Inertia::default(), 
            },
            // Fix E0599: Methode heißt in location.rs jetzt to_msg()
            location: self.location.to_msg(),
            quantity: self.quantity as u32, 
            uuid: self.uid.clone(),
        }
    }

    pub fn from_msg(msg: &ItemMsg) -> Self {
        Self {
            name: msg.part.name.clone(),
            uid: msg.part.uuid.clone(),
            quantity: msg.quantity as i32, 
            location: WorldLocation::from_msg(&msg.location),
        }
    }

    pub fn to_basyx_json(&self) -> Value {
        let id_short = format!("item_{}", sanitize_id(&self.uid));
        json!({
            "idShort": id_short,
            "category": "WorldItem",
            "modelType": "SubmodelElementCollection",
            "value": [
                {
                    "idShort": "name",
                    "modelType": "Property",
                    "valueType": "string",
                    "value": self.name
                },
                {
                    "idShort": "uid",
                    "modelType": "Property",
                    "valueType": "string",
                    "value": self.uid
                },
                {
                    "idShort": "quantity",
                    "modelType": "Property",
                    "valueType": "integer",
                    "value": self.quantity
                },
                // Fix E0599
                self.location.to_basyx_json() 
            ]
        })
    }
}