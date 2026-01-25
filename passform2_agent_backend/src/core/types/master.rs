use serde::{Serialize, Deserialize};
use serde_json::{json, Value};
use uuid::Uuid;
use crate::util::sanitize_id;
use crate::types::bay::BaySubmodel; // Unser vorheriges Bay-Submodell

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Master {
    pub unique_id: String,
    pub name: String,
    pub bays: Vec<BaySubmodel>,
}

impl Master {
    pub fn new(unique_id: Option<String>, name: Option<String>) -> Self {
        Self {
            unique_id: unique_id.unwrap_or_else(|| Uuid::new_v4().to_string()),
            name: sanitize_id(&name.unwrap_or_else(|| "PassForM_Master".to_string())),
            bays: Vec::new(),
        }
    }

    /// Erstellt eine neue Bay und fügt sie dem Master hinzu
    pub fn create_bay(&mut self, bay_id: &str, origin: [f64; 3]) {
        let bay_uuid = Uuid::new_v4().to_string();
        let bay_name = format!("Bay{}", bay_id);
        let new_bay = BaySubmodel::new(&bay_uuid, &bay_name, origin, false);
        self.bays.push(new_bay);
    }

    /// Gibt alle Submodelle vom Typ "passform_bay" zurück
    pub fn get_bays(&self) -> &Vec<BaySubmodel> {
        &self.bays
    }

    /// Generiert die komplette AAS-Struktur als JSON für den BaSyx-Server
    pub fn to_basyx_json(&self) -> Value {
        let mut submodels: Vec<Value> = self.bays
            .iter()
            .map(|b| b.to_basyx_json())
            .collect();

        // Füge das TechnicalData Submodell des Masters hinzu
        submodels.push(json!({
            "idShort": "TechnicalData",
            "identification": {
                "id": format!("{}_TD", self.unique_id),
                "idType": "CUSTOM"
            },
            "modelType": "Submodel",
            "submodelElements": []
        }));

        json!({
            "idShort": self.name,
            "identification": {
                "id": self.unique_id,
                "idType": "CUSTOM"
            },
            "category": "passform_master",
            "modelType": "AssetAdministrationShell",
            "asset": {
                "idShort": "MasterAsset",
                "identification": {
                    "id": format!("{}_ASSET", self.unique_id),
                    "idType": "CUSTOM"
                },
                "assetKind": "INSTANCE"
            },
            "submodels": submodels
        })
    }
}