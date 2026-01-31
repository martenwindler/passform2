use serde_json::Value;
use reqwest::blocking::Client;
use std::time::Duration;

pub struct RestServer {
    registry_url: String,
    repository_url: String,
    client: Client,
}

impl RestServer {
    pub fn new(host: &str, registry_port: u16, repository_port: u16, timeout_secs: u32) -> Self {
        Self {
            registry_url: format!("http://{}:{}/registry/shell-descriptors", host, registry_port),
            repository_url: format!("http://{}:{}/shells", host, repository_port),
            client: Client::builder()
                .timeout(Duration::from_secs(timeout_secs as u64))
                .build()
                .unwrap_or_default(),
        }
    }

    /// Registriert eine AAS in der Registry
    pub fn add_aas(&self, aas_json: Value) -> Result<(), Box<dyn std::error::Error>> {
        let res = self.client.post(&self.registry_url)
            .json(&aas_json)
            .send()?;

        if res.status().is_success() {
            Ok(())
        } else {
            Err(format!("BaSyx Registry Error: {}", res.status()).into())
        }
    }

    /// Fügt ein Submodell zu einer existierenden AAS hinzu
    pub fn add_submodel(&self, aas_id: &str, submodel_json: Value) -> Result<(), Box<dyn std::error::Error>> {
        let url = format!("{}/{}/submodels", self.repository_url, urlencoding::encode(aas_id));
        let res = self.client.post(&url)
            .json(&submodel_json)
            .send()?;

        if res.status().is_success() {
            Ok(())
        } else {
            Err(format!("BaSyx Repository Error: {}", res.status()).into())
        }
    }

    /// Löscht eine AAS aus der Registry (Cleanup)
    pub fn remove_aas(&self, aas_id: &str) -> Result<(), Box<dyn std::error::Error>> {
        let url = format!("{}/{}", self.registry_url, urlencoding::encode(aas_id));
        self.client.delete(&url).send()?;
        Ok(())
    }
}