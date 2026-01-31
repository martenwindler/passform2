use serde_json::{json, Value};
use crate::core::util::helper::sanitize_id;
use crate::core::util::watchdog::Watchdog;
use std::sync::{Arc, Mutex};

/// Repräsentiert die gesamte Bay AAS
pub struct Bay {
    pub unique_id: String,
    pub name: String,
    pub origin: [f64; 3],
    pub watchdog: Option<Watchdog>,
    // Wir speichern den internen Status als JSON-Value für die BaSyx-Kompatibilität
    status: Arc<Mutex<Value>>,
}

impl Bay {
    pub fn new(unique_id: &str, name: &str, origin: [f64; 3]) -> Self {
        let sanitized_name = sanitize_id(name);
        
        // Initialer Status-Submodel im BaSyx-Format
        let status = json!({
            "idShort": "BayStatus",
            "identification": {
                "id": format!("{}_STATUS", unique_id),
                "idType": "CUSTOM"
            },
            "submodelElements": [
                { "idShort": "occupation", "modelType": "Property", "valueType": "boolean", "value": false },
                { "idShort": "module", "modelType": "Property", "valueType": "string", "value": "" },
                { "idShort": "timeout", "modelType": "Property", "valueType": "boolean", "value": false },
                {
                    "idShort": "SupplyStatus",
                    "modelType": "SubmodelElementCollection",
                    "value": [
                        { "idShort": "dc_24v", "modelType": "Property", "valueType": "boolean", "value": true },
                        { "idShort": "ac_230v", "modelType": "Property", "valueType": "boolean", "value": false }
                    ]
                }
            ]
        });

        Self {
            unique_id: unique_id.to_string(),
            name: sanitized_name,
            origin,
            watchdog: None,
            status: Arc::new(Mutex::new(status)),
        }
    }

    /// Erzeugt das vollständige BaSyx-JSON für den Server-Upload
    pub fn to_aas_json(&self) -> Value {
        let status = self.status.lock().unwrap();
        json!({
            "idShort": self.name,
            "identification": {
                "id": self.unique_id,
                "idType": "CUSTOM"
            },
            "category": "passform_bay",
            "asset": {
                "idShort": "BayAsset",
                "identification": {
                    "id": format!("{}_ASSET", self.unique_id),
                    "idType": "CUSTOM"
                },
                "assetKind": "INSTANCE"
            },
            "submodels": [
                status.clone(),
                self.create_technical_data_json()
            ]
        })
    }

    fn create_technical_data_json(&self) -> Value {
        json!({
            "idShort": "TechnicalData",
            "identification": {
                "id": format!("{}_TD", self.unique_id),
                "idType": "CUSTOM"
            },
            "submodelElements": [
                {
                    "idShort": "Origin",
                    "modelType": "SubmodelElementCollection",
                    "value": [
                        { "idShort": "x", "modelType": "Property", "valueType": "double", "value": self.origin[0] },
                        { "idShort": "y", "modelType": "Property", "valueType": "double", "value": self.origin[1] },
                        { "idShort": "z", "modelType": "Property", "valueType": "double", "value": self.origin[2] }
                    ]
                }
            ]
        })
    }

    pub fn is_free(&self) -> bool {
        let status = self.status.lock().unwrap();
        // Navigiere im JSON zum 'occupation' Feld
        status["submodelElements"]
            .as_array()
            .and_then(|elems| elems.iter().find(|e| e["idShort"] == "occupation"))
            .and_then(|occ| occ["value"].as_bool())
            .map(|val| !val)
            .unwrap_or(true)
    }

    pub fn occupy(&mut self, module_uuid: &str) {
        let mut status = self.status.lock().unwrap();
        if let Some(elems) = status["submodelElements"].as_array_mut() {
            for e in elems {
                if e["idShort"] == "occupation" { e["value"] = json!(true); }
                if e["idShort"] == "module" { e["value"] = json!(module_uuid); }
                if e["idShort"] == "timeout" { e["value"] = json!(false); }
            }
        }
        
        // Watchdog-Logik
        let status_ptr = Arc::clone(&self.status);
        self.watchdog = Some(Watchdog::new(2.0, move || {
            let mut s = status_ptr.lock().unwrap();
            if let Some(elems) = s["submodelElements"].as_array_mut() {
                if let Some(t) = elems.iter_mut().find(|e| e["idShort"] == "timeout") {
                    t["value"] = json!(true);
                }
            }
            eprintln!("Watchdog: Bay module timed out.");
        }));
    }

    pub fn free(&mut self) {
        if let Some(wd) = self.watchdog.take() {
            wd.stop();
        }
        
        let mut status = self.status.lock().unwrap();
        if let Some(elems) = status["submodelElements"].as_array_mut() {
            for e in elems {
                if e["idShort"] == "occupation" { e["value"] = json!(false); }
                if e["idShort"] == "module" { e["value"] = json!(""); }
                if e["idShort"] == "timeout" { e["value"] = json!(false); }
            }
        }
    }
}