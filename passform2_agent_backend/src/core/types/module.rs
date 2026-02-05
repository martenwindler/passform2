use serde::{Serialize, Deserialize};
use serde_json::{json, Value};
use crate::core::util::util::sanitize_id;

/// Repräsentiert die Versorgungsanforderungen
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Supply {
    pub dc_24v: bool,
    pub ac_230v: bool,
    pub vacuum: bool,
}

impl Default for Supply {
    fn default() -> Self {
        Self { dc_24v: true, ac_230v: false, vacuum: false }
    }
}

/// Die Haupt-Struktur für ein PassForM Modul
pub struct Module {
    pub uuid: String,
    pub name: String,
    pub properties: Option<Value>, // Dynamische Daten aus der YAML
    pub thumbnail_path: Option<String>,
}

impl Module {
    pub fn new(uuid: &str, name: &str, properties: Option<Value>, thumbnail: Option<String>) -> Self {
        Self {
            uuid: uuid.to_string(),
            name: sanitize_id(name),
            properties,
            thumbnail_path: thumbnail,
        }
    }

    /// Generiert das vollständige AAS-JSON für BaSyx
    pub fn to_aas_json(&self) -> Value {
        json!({
            "idShort": self.name,
            "identification": {
                "id": self.uuid,
                "idType": "CUSTOM"
            },
            "category": "module",
            "modelType": "AssetAdministrationShell",
            "asset": {
                "idShort": "ModuleAsset",
                "identification": {
                    "id": format!("{}_ASSET", self.uuid),
                    "idType": "CUSTOM"
                },
                "assetKind": "INSTANCE"
            },
            "submodels": [
                self.create_technical_data_submodel(),
                self.create_inventory_submodel()
            ]
        })
    }

    fn create_technical_data_submodel(&self) -> Value {
        let mut elements = Vec::new();

        // 1. Thumbnail als Blob hinzufügen (falls vorhanden)
        if let Some(_path) = &self.thumbnail_path {
            // In einer echten Implementierung würden wir hier die Datei einlesen
            elements.push(json!({
                "idShort": "thumbnail",
                "modelType": "Blob",
                "contentType": "image/png",
                "value": "BASE64_ENCODED_DATA_HERE" 
            }));
        }

        // 2. Supply Characteristics
        if let Some(props) = &self.properties {
            if let Some(supply) = props.get("TechnicalData").and_then(|td| td.get("Supply")) {
                elements.push(self.map_supply_to_basyx(supply));
            }
            
            // 3. Physical Properties (Mass, Width, etc.)
            if let Some(phys) = props.get("TechnicalData").and_then(|td| td.get("PhysicalProperties")) {
                elements.push(json!({
                    "idShort": "PhysicalProperties",
                    "modelType": "SubmodelElementCollection",
                    "value": phys
                }));
            }
        }

        json!({
            "idShort": "TechnicalData",
            "identification": { "id": format!("{}_TD", self.uuid), "idType": "CUSTOM" },
            "modelType": "Submodel",
            "submodelElements": elements
        })
    }

    fn map_supply_to_basyx(&self, supply_props: &Value) -> Value {
        let mut values = Vec::new();
        let map = [("24vdc", "dc_24v"), ("230vac", "ac_230v"), ("vacuum", "vacuum")];

        for (yaml_key, basyx_id) in map {
            if supply_props.get(yaml_key).and_then(|v| v.as_bool()).unwrap_or(false) {
                values.push(json!({
                    "idShort": basyx_id,
                    "modelType": "Property",
                    "valueType": "boolean",
                    "value": true
                }));
            }
        }

        json!({
            "idShort": "SupplyCharacteristics",
            "modelType": "SubmodelElementCollection",
            "value": values
        })
    }

    fn create_inventory_submodel(&self) -> Value {
        let mut inventory_elements = Vec::new();
        
        if let Some(props) = &self.properties {
            if let Some(inv) = props.get("Inventory") {
                inventory_elements = inv.as_array().cloned().unwrap_or_default();
            }
        }

        json!({
            "idShort": "inventory",
            "identification": { "id": format!("{}_Inventory", self.uuid), "idType": "CUSTOM" },
            "modelType": "Submodel",
            "submodelElements": [{
                "idShort": "Inventory",
                "modelType": "SubmodelElementCollection",
                "value": inventory_elements
            }]
        })
    }
}