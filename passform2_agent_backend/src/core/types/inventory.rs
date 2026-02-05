use serde::{Serialize, Deserialize};
use crate::core::types::location::WorldLocation;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct WorldItem {
    pub name: String,
    pub uid: String,
    pub quantity: i32,
    pub location: WorldLocation,
}

impl WorldItem {
    pub fn new(name: &str, uid: &str, quantity: i32, location: Option<WorldLocation>) -> Self {
        Self {
            name: name.to_string(),
            uid: uid.to_string(),
            quantity: quantity.max(0), 
            location: location.unwrap_or_default(),
        }
    }
}