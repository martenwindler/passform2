use serde::{Deserialize, Serialize};
use std::time::Instant;
use crate::core::util::util::sanitize_id;
use crate::core::types::Status;

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Bay {
    pub unique_id: String,
    pub name: String,
    pub origin: [f64; 3],
    pub is_virtual: bool, 
    pub status: Status,
    pub current_rfid: Option<String>,
    pub occupation: bool,
    pub module_uuid: String,
    pub has_timeout: bool,
    
    #[serde(skip, default = "Instant::now")] 
    pub last_update: Instant, 
}

impl Bay {
    pub fn new(unique_id: &str, name: &str, origin: [f64; 3], is_virtual: bool) -> Self {
        Self {
            unique_id: unique_id.to_string(),
            name: sanitize_id(name),
            origin,
            is_virtual,
            status: Status::Ok,
            current_rfid: None,
            occupation: false,
            module_uuid: String::new(),
            has_timeout: false,
            last_update: Instant::now(),
        }
    }
}