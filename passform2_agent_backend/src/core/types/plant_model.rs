use serde::{Serialize, Deserialize};
use uuid::Uuid;
use crate::core::util::util::sanitize_id;
use crate::core::types::Bay;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PlantModel {
    pub unique_id: String,
    pub name: String,
    pub bays: Vec<Bay>,
}

impl PlantModel {
    pub fn new(unique_id: Option<String>, name: Option<String>) -> Self {
        Self {
            unique_id: unique_id.unwrap_or_else(|| Uuid::new_v4().to_string()),
            name: sanitize_id(&name.unwrap_or_else(|| "PassForM_Master".to_string())),
            bays: Vec::new(),
        }
    }

    pub fn create_bay(&mut self, bay_id: &str, origin: [f64; 3]) {
        let bay_uuid = Uuid::new_v4().to_string();
        let bay_name = format!("{}", bay_id);
        let new_bay = Bay::new(&bay_uuid, &bay_name, origin, false);
        self.bays.push(new_bay);
    }
}