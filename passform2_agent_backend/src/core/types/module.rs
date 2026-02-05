use serde::{Serialize, Deserialize};
use serde_json::Value;
use std::collections::HashMap;
use crate::core::util::util::sanitize_id;

/// Repräsentiert die Versorgungsanforderungen eines Moduls
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct Supply {
    pub dc_24v: bool,
    pub ac_230v: bool,
    pub vacuum: bool,
}

impl Default for Supply {
    fn default() -> Self {
        Self { 
            dc_24v: true, 
            ac_230v: false, 
            vacuum: false 
        }
    }
}

/// Die reine Daten-Struktur für ein PassForM Modul (Dumb Data)
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Module {
    pub uuid: String,
    pub name: String,
    pub supply: Supply,
    /// Physische Eigenschaften wie Masse, Breite etc.
    pub physical_properties: HashMap<String, Value>, 
    /// Liste von Inventar-Gegenständen, die das Modul hält
    pub inventory: Vec<Value>,
    pub thumbnail_path: Option<String>,
    /// Rohdaten aus der YAML-Konfiguration für Spezialfälle
    pub raw_properties: Option<Value>, 
}

impl Module {
    /// Erstellt ein neues, leeres Modul
    pub fn new(uuid: &str, name: &str) -> Self {
        Self {
            uuid: uuid.to_string(),
            name: sanitize_id(name),
            supply: Supply::default(),
            physical_properties: HashMap::new(),
            inventory: Vec::new(),
            thumbnail_path: None,
            raw_properties: None,
        }
    }

    /// Hilfsfunktion zum schnellen Setzen der Versorgung
    pub fn with_supply(mut self, dc_24v: bool, ac_230v: bool, vacuum: bool) -> Self {
        self.supply = Supply { dc_24v, ac_230v, vacuum };
        self
    }
}