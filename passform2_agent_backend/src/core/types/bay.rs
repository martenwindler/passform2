use serde::{Serialize, Deserialize};
use serde_json::{json, Value};
use crate::util::sanitize_id;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct BaySubmodel {
    pub unique_id: String,
    pub name: String,
    pub origin: [f64; 3],
    pub virtual_bay: bool,
}

impl BaySubmodel {
    pub fn new(unique_id: &str, name: &str, origin: [f64; 3], virtual_bay: bool) -> Self {
        Self {
            unique_id: unique_id.to_string(),
            name: sanitize_id(name),
            origin,
            virtual_bay,
        }
    }

    /// Generiert das BaSyx-konforme JSON-Submodell
    pub fn to_basyx_json(&self) -> Value {
        let category = if self.virtual_bay { "virtual_bay" } else { "passform_bay" };
        
        let mut submodel = json!({
            "idShort": self.name,
            "identification": {
                "id": self.unique_id,
                "idType": "CUSTOM"
            },
            "category": category,
            "modelType": "Submodel",
            "submodelElements": []
        });

        // Wenn nicht virtuell, füge Geometrie und Versorgung hinzu
        if !self.virtual_bay {
            let elements = submodel["submodelElements"].as_array_mut().unwrap();
            elements.push(self.create_geometry_collection());
            elements.push(self.create_supply_collection());
        }

        submodel
    }

    fn create_geometry_collection(&self) -> Value {
        json!({
            "idShort": "Origin",
            "modelType": "SubmodelElementCollection",
            "description": [
                {"language": "en-us", "text": "Description of the bays origin."},
                {"language": "de", "text": "Beschreibung des Ursprungs der Bucht."}
            ],
            "value": [
                self.create_property("x", self.origin[0], "x-position of the origin in m.", "x-Position des Ursprungs in m."),
                self.create_property("y", self.origin[1], "y-position of the origin in m.", "y-Position des Ursprungs in m."),
                self.create_property("z", self.origin[2], "z-position of the origin in m.", "z-Position des Ursprungs in m.")
            ]
        })
    }

    fn create_supply_collection(&self) -> Value {
        json!({
            "idShort": "SupplyStatus",
            "modelType": "SubmodelElementCollection",
            "description": [
                {"language": "en-us", "text": "List of all utilities currently active."},
                {"language": "de", "text": "Liste aller aktuell aktiven Versorgungen."}
            ],
            "value": [
                self.create_bool_property("dc_24v", true, "24V DC.", "24V DC."),
                self.create_bool_property("ac_230v", false, "230V AC.", "230V AC.")
            ]
        })
    }

    fn create_property(&self, id: &str, value: f64, desc_en: &str, desc_de: &str) -> Value {
        json!({
            "idShort": id,
            "modelType": "Property",
            "valueType": "double",
            "value": value,
            "description": [
                {"language": "en-us", "text": desc_en},
                {"language": "de", "text": desc_de}
            ]
        })
    }

    fn create_bool_property(&self, id: &str, value: bool, desc_en: &str, desc_de: &str) -> Value {
        json!({
            "idShort": id,
            "modelType": "Property",
            "valueType": "boolean",
            "value": value,
            "description": [
                {"language": "en-us", "text": desc_en},
                {"language": "de", "text": desc_de}
            ]
        })
    }
}