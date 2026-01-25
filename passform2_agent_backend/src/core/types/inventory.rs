use serde::{Serialize, Deserialize};
use serde_json::{json, Value};
use crate::util::sanitize_id;
use crate::types::location::Location; // Deine Location-Portierung
use passform_msgs::msg::{Item as ItemMsg, Part as PartMsg};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Item {
    pub name: String,
    pub uid: String,
    pub quantity: i32,
    pub location: Location,
}

impl Item {
    pub fn new(name: &str, uid: &str, quantity: i32, location: Option<Location>) -> Self {
        assert!(!uid.is_empty(), "unique id must not be empty");
        Self {
            name: name.to_string(),
            uid: uid.to_string(),
            quantity: if quantity < 0 { 0 } else { quantity },
            location: location.unwrap_or_default(),
        }
    }

    /// Entspricht self.add(other) in Python
    pub fn add(&mut self, other: &mut Item) -> Result<(), String> {
        if self.uid != other.uid {
            return Err(format!(
                "Adding quantity to not matching item IDs: {} to {}",
                other.uid, self.uid
            ));
        }
        self.quantity += other.quantity;
        other.quantity = 0; // "use all" Logik aus dem Python-Original
        Ok(())
    }

    /// Entspricht self.use(quantity) in Python
    pub fn use_qty(&mut self, mut quantity: i32) -> Item {
        if quantity < 0 { quantity = 0; }
        if quantity > self.quantity {
            quantity = 0;
        }
        
        let mut new_item = self.clone();
        new_item.quantity = quantity;
        self.quantity -= quantity;
        new_item
    }

    /// Prüft ob eine Location gesetzt ist (UID nicht leer)
    pub fn has_location(&self) -> bool {
        !self.location.get_aoi_uid().is_empty()
    }

    /// Konvertiert in eine ROS-Nachricht
    pub fn to_msg(&self) -> ItemMsg {
        ItemMsg {
            part: PartMsg {
                uuid: self.uid.clone(),
                name: self.name.clone(),
            },
            location: self.location.to_msg(),
            quantity: self.quantity,
            uuid: self.uid.clone(),
        }
    }

    /// Erzeugt ein Item aus einer ROS-Nachricht
    pub fn from_msg(msg: &ItemMsg) -> Self {
        Self {
            name: msg.part.name.clone(),
            uid: msg.part.uuid.clone(),
            quantity: msg.quantity,
            location: Location::from_msg(&msg.location),
        }
    }

    /// Erzeugt das BaSyx-JSON Format (SubmodelElementCollection)
    pub fn to_basyx_json(&self) -> Value {
        let id_short = format!("item_{}", sanitize_id(&self.uid));
        json!({
            "idShort": id_short,
            "category": "Item",
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
                self.location.to_basyx_json() // Location muss diese Methode implementieren
            ]
        })
    }
}