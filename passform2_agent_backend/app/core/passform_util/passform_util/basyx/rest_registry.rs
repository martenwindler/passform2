use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use reqwest::blocking::Client;
use std::time::Duration;
use crate::util::sanitize_id;

#[derive(Debug, Serialize, Deserialize)]
pub struct Endpoint {
    pub address: String,
    #[serde(rename = "type")]
    pub protocol_type: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Descriptor {
    #[serde(rename = "identification")]
    pub identification: crate::basyx::Identifier,
    #[serde(rename = "idShort")]
    pub id_short: String,
    #[serde(rename = "modelType")]
    pub model_type: HashMap<String, String>,
    pub endpoints: Vec<Endpoint>,
}

pub struct RestRegistry {
    url: String,
    client: Client,
    timeout: Duration,
}

impl RestRegistry {
    pub fn new(host: &str, port: u16, timeout_sec: u64) -> Self {
        Self {
            url: format!("http://{}:{}/registry/api/v1/registry/", host, port),
            client: Client::new(),
            timeout: Duration::from_secs(timeout_sec),
        }
    }

    /// Hilfsmethode für generische GET-Anfragen
    fn get_request(&self, full_url: &str) -> Result<Value, Box<dyn std::error::Error>> {
        let resp = self.client.get(full_url)
            .timeout(self.timeout)
            .send()?;
        
        if resp.status().is_success() {
            Ok(resp.json()?)
        } else {
            Err(format!("GET failed with status {}: {}", resp.status(), full_url).into())
        }
    }

    /// Fügt eine AAS zur Registry hinzu (AssetAdministrationShellDescriptor)
    pub fn add_aas(&self, id: &str, id_short: &str, endpoint_url: &str) -> Result<(), Box<dyn std::error::Error>> {
        let data = json!({
            "identification": {
                "id": id,
                "idType": "IRI" // oder dynamisch anpassen
            },
            "idShort": id_short,
            "modelType": { "name": "AssetAdministrationShellDescriptor" },
            "endpoints": [{
                "address": endpoint_url,
                "type": "http"
            }]
        });

        let url = format!("{}{}", self.url, sanitize_id(id));
        let resp = self.client.put(&url)
            .json(&data)
            .timeout(self.timeout)
            .send()?;

        if resp.status().is_success() { Ok(()) } else { Err("Failed to add AAS to registry".into()) }
    }

    /// Fügt ein Submodel zur Registry hinzu
    pub fn add_submodel(&self, aas_id: &str, submodel_id: &str, id_short: &str, endpoint_url: &str) -> Result<(), Box<dyn std::error::Error>> {
        let data = json!({
            "identification": {
                "id": submodel_id,
                "idType": "IRI"
            },
            "idShort": id_short,
            "modelType": { "name": "SubmodelDescriptor" },
            "endpoints": [{
                "address": endpoint_url,
                "type": "http"
            }]
        });

        let url = format!("{}{}/submodels/{}", self.url, sanitize_id(aas_id), sanitize_id(submodel_id));
        let resp = self.client.put(&url)
            .json(&data)
            .timeout(self.timeout)
            .send()?;

        if resp.status().is_success() { Ok(()) } else { Err("Failed to add Submodel to registry".into()) }
    }

    /// Entfernt eine AAS aus der Registry
    pub fn remove_aas(&self, aas_id: &str) -> Result<(), Box<dyn std::error::Error>> {
        let url = format!("{}{}", self.url, sanitize_id(aas_id));
        let resp = self.client.delete(&url).timeout(self.timeout).send()?;
        
        if resp.status().is_success() { Ok(()) } else { Err("Registry DELETE failed".into()) }
    }

    /// Sucht ein Objekt anhand der ID in der Registry (globaler Scan)
    pub fn find_object_endpoint(&self, id: &str) -> Result<String, Box<dyn std::error::Error>> {
        // GET auf die Liste aller Shells
        let shells = self.get_request(&self.url)?;
        
        if let Some(shells_array) = shells.as_array() {
            for shell in shells_array {
                // Prüfe AAS ID
                if shell["identification"]["id"] == id {
                    return Ok(shell["endpoints"][0]["address"].as_str().unwrap_or("").to_string());
                }
                
                // Prüfe Submodels innerhalb der Shell
                if let Some(submodels) = shell["submodels"].as_array() {
                    for sm in submodels {
                        if sm["identification"]["id"] == id {
                            return Ok(sm["endpoints"][0]["address"].as_str().unwrap_or("").to_string());
                        }
                    }
                }
            }
        }
        Err("Object not found in registry".into())
    }
}