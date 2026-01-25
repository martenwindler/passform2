use serde::{Deserialize, Serialize};
use reqwest::blocking::Client;
use std::collections::HashMap;
use std::time::Duration;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Identifier {
    pub id: String,
    #[serde(rename = "idType")]
    pub id_type: String,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct AssetAdministrationShell {
    pub identification: Identifier,
    #[serde(rename = "idShort")]
    pub id_short: String,
    // Weitere Felder nach Bedarf
}

pub struct BaSyxClient {
    pub repo_url: String,
    pub registry_url: String,
    client: Client,
    timeout: Duration,
}

impl BaSyxClient {
    pub fn new(host: &str, repo_port: u16, registry_port: u16, timeout_sec: u64) -> Self {
        Self {
            repo_url: format!("http://{}:{}/aasServer/shells/", host, repo_port),
            registry_url: format!("http://{}:{}/registry/api/v1/registry/", host, registry_port),
            client: Client::new(),
            timeout: Duration::from_secs(timeout_sec),
        }
    }

    /// Entspricht add_aas im Python-Code
    pub fn add_aas(&self, aas: &AssetAdministrationShell) -> Result<(), Box<dyn std::error::Error>> {
        let id_clean = crate::util::sanitize_id(&aas.identification.id);
        let url = format!("{}{}", self.repo_url, id_clean);

        // 1. PUT zum AAS Server
        let res = self.client.put(&url)
            .json(aas)
            .timeout(self.timeout)
            .send()?;

        if !res.status().is_success() {
            return Err(format!("AAS Server error: {}", res.status()).into());
        }

        // 2. Registry Eintrag (vereinfacht)
        self.register_aas(aas, &url)?;

        Ok(())
    }

    fn register_aas(&self, aas: &AssetAdministrationShell, endpoint: &str) -> Result<(), Box<dyn std::error::Error>> {
        let id_clean = crate::util::sanitize_id(&aas.identification.id);
        let reg_url = format!("{}{}", self.registry_url, id_clean);

        // Hier würde ein Descriptor-Objekt gebaut werden wie in rest_registry.py
        let mut descriptor = serde_json::to_value(aas)?;
        descriptor["endpoints"] = serde_json::json!([{
            "address": endpoint,
            "type": "http"
        }]);

        let res = self.client.put(&reg_url)
            .json(&descriptor)
            .send()?;

        if res.status().is_success() { Ok(()) } else { Err("Registry error".into()) }
    }

    pub fn remove_aas(&self, aas_id: &str) -> Result<(), Box<dyn std::error::Error>> {
        let id_clean = crate::util::sanitize_id(aas_id);
        
        // Löschen vom Server
        self.client.delete(format!("{}{}", self.repo_url, id_clean)).send()?;
        // Löschen aus Registry
        self.client.delete(format!("{}{}", self.registry_url, id_clean)).send()?;
        
        Ok(())
    }
}