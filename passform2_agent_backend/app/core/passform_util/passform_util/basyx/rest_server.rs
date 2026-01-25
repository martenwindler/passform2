use reqwest::blocking::Client;
use serde_json::{json, Value};
use std::time::Duration;
use crate::util::sanitize_id;
use crate::basyx::rest_registry::RestRegistry;
use crate::basyx::conversion::SubmodelElement;

pub struct RestServer {
    host: String,
    port: u16,
    aas_server_url: String,
    timeout: Duration,
    pub registry: RestRegistry,
}

impl RestServer {
    pub fn new(
        registry_host: &str,
        server_port: u16,
        registry_port: u16,
        timeout_sec: u64,
    ) -> Self {
        // Extrahiere Host aus Registry-String (z.B. "localhost:8082")
        let host_only = registry_host.split(':').next().unwrap_or("localhost");
        let timeout = Duration::from_secs(timeout_sec);

        Self {
            host: format!("http://{}", host_only),
            port: server_port,
            aas_server_url: format!("http://{}:{}/aasServer/shells/", host_only, server_port),
            timeout,
            registry: RestRegistry::new(host_only, registry_port, timeout_sec),
        }
    }

    /// Hilfsmethode für generische Fehlerbehandlung bei Responses
    fn check_status(resp: reqwest::blocking::Response, msg: &str) -> Result<reqwest::blocking::Response, Box<dyn std::error::Error>> {
        if resp.status().is_success() {
            Ok(resp)
        } else {
            let status = resp.status();
            let body = resp.text().unwrap_or_else(|_| "No body".into());
            Err(format!("{}: Status {}, Body: {}", msg, status, body).into())
        }
    }

    /// Fügt eine komplette Shell inklusive Submodellen hinzu
    pub fn add_aas(&self, aas_json: Value) -> Result<(), Box<dyn std::error::Error>> {
        let aas_id = aas_json["identification"]["id"].as_str()
            .ok_or("AAS ID missing in JSON")?;
        let id_clean = sanitize_id(aas_id);

        // In Python gab es 'seperate_submodel_from_aas'. 
        // Hier arbeiten wir direkt mit dem JSON-Value für maximale Flexibilität.
        let mut shell_only = aas_json.clone();
        let submodels = shell_only["submodels"].take();

        // 1. Shell hinzufügen
        let client = Client::new();
        let resp = client.put(format!("{}{}", self.aas_server_url, id_clean))
            .json(&shell_only)
            .timeout(self.timeout)
            .send()?;
        Self::check_status(resp, "Failed to add shell")?;

        // 2. In Registry eintragen
        let id_short = shell_only["idShort"].as_str().unwrap_or("UnnamedAAS");
        let endpoint = format!("{}{}/aas/", self.aas_server_url, id_clean);
        self.registry.add_aas(aas_id, id_short, &endpoint)?;

        // 3. Submodelle einzeln hinzufügen
        if let Some(sm_array) = submodels.as_array() {
            for sm in sm_array {
                self.add_submodel(aas_id, sm.clone())?;
            }
        }

        Ok(())
    }

    pub fn add_submodel(&self, aas_id: &str, submodel: Value) -> Result<(), Box<dyn std::error::Error>> {
        let aas_id_clean = sanitize_id(aas_id);
        let sm_id_short = submodel["idShort"].as_str().ok_or("Submodel idShort missing")?;
        let sm_id = submodel["identification"]["id"].as_str().ok_or("Submodel identification missing")?;
        
        let url = format!("{}{}/aas/submodels/{}", self.aas_server_url, aas_id_clean, sm_id_short);
        
        let client = Client::new();
        let resp = client.put(&url)
            .json(&submodel)
            .timeout(self.timeout)
            .send()?;
        Self::check_status(resp, "Failed to add submodel")?;

        // Registry Update
        let endpoint = format!("{}/submodel", url);
        self.registry.add_submodel(aas_id, sm_id, sm_id_short, &endpoint)?;

        Ok(())
    }

    pub fn remove_aas(&self, aas_id: &str) -> Result<(), Box<dyn std::error::Error>> {
        let id_clean = sanitize_id(aas_id);
        let client = Client::new();
        
        let resp = client.delete(format!("{}{}", self.aas_server_url, id_clean))
            .timeout(self.timeout)
            .send()?;
        
        if resp.status().is_success() {
            self.registry.remove_aas(aas_id)?;
            Ok(())
        } else {
            Err(format!("Delete AAS failed: {}", resp.status()).into())
        }
    }

    /// Wert eines Elements setzen (Property, etc.)
    pub fn set_value(
        &self,
        aas_id: &str,
        submodel_id_short: &str,
        value_id_short: &str,
        value: Value
    ) -> Result<(), Box<dyn std::error::Error>> {
        let url = format!(
            "{}{}/aas/submodels/{}/submodel/submodelElements/{}/value",
            self.aas_server_url,
            sanitize_id(aas_id),
            sanitize_id(submodel_id_short),
            sanitize_id(value_id_short)
        );

        let client = Client::new();
        let resp = client.put(&url)
            .json(&value)
            .timeout(self.timeout)
            .send()?;

        Self::check_status(resp, "Failed to set value")?;
        Ok(())
    }
}