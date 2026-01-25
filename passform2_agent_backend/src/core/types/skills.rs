use serde::{Serialize, Deserialize};
use serde_json::{json, Value};
use uuid::Uuid;
use crate::util::sanitize_id;
use std::collections::HashMap;

/// Alle verfügbaren Fähigkeitstypen (identisch zu Python Enum)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[repr(i32)]
pub enum SkillType {
    Stop = 0, Move = 1, MoveTcp = 2, MoveBody = 3,
    Grasp = 10, Pressure = 12, Position = 13, Release = 14,
    ToolGet = 20, ToolUse = 21, ToolPut = 22, Store = 23, Provide = 24,
    AssemblyPickPlace = 30, AssemblyScrew = 31, AssemblyMount = 32, AssemblyPlug = 33,
}

/// Eine BaSyx-Operation innerhalb eines Skills
pub struct Operation {
    pub id_short: String,
    pub input_required: Vec<Value>,
    pub input_optional: Vec<Value>,
    pub output: Vec<Value>,
}

impl Operation {
    /// Prüft, ob ein ROS-Task alle erforderlichen Parameter liefert
    pub fn can_perform(&self, provided_params: &[String]) -> bool {
        // Alle erforderlichen id_shorts der Operation müssen in den provided_params sein
        self.input_required.iter().all(|req| {
            if let Some(id) = req.get("idShort").and_then(|id| id.as_str()) {
                provided_params.contains(&id.to_string())
            } else {
                false
            }
        })
    }

    pub fn to_basyx_json(&self) -> Value {
        // Hier bauen wir das komplexe BaSyx-Operation-Schema
        json!({
            "idShort": self.id_short,
            "modelType": "Operation",
            "inputVariables": self.input_required.iter().map(|v| self.wrap_variable(v, "required")).collect::<Vec<_>>(),
            "outputVariables": self.output.iter().map(|v| self.wrap_variable(v, "output")).collect::<Vec<_>>()
        })
    }

    fn wrap_variable(&self, data: &Value, category: &str) -> Value {
        json!({
            "value": {
                "idShort": data["id_short"],
                "category": category,
                "modelType": "Property",
                "valueType": data["value_type"]
            }
        })
    }
}

/// Das Submodell für einen Skill
pub struct Skill {
    pub uuid: String,
    pub id_short: String,
    pub skill_type: SkillType,
    pub driver_topic: String,
    pub properties: Value,
    pub operations: Vec<Operation>,
}

impl Skill {
    pub fn new(skill_type: SkillType, data: Value) -> Self {
        let uuid = Uuid::new_v4().to_string();
        let id_short = data.get("id_short")
            .and_then(|v| v.as_str())
            .unwrap_or_else(|| match skill_type { _ => "Skill" }); // Vereinfacht

        Self {
            uuid,
            id_short: sanitize_id(id_short),
            skill_type,
            driver_topic: data["driver_topic"].as_str().unwrap_or_default().to_string(),
            properties: data["properties"].clone(),
            operations: Vec::new(), // Würde aus 'input'/'output' Daten generiert
        }
    }

    pub fn to_aas_submodel_json(&self) -> Value {
        json!({
            "idShort": self.id_short,
            "identification": { "id": self.uuid, "idType": "CUSTOM" },
            "category": "Skill",
            "modelType": "Submodel",
            "submodelElements": [
                {
                    "idShort": "skilltype",
                    "modelType": "Property",
                    "valueType": "int",
                    "value": self.skill_type as i32
                },
                {
                    "idShort": "driver",
                    "modelType": "Property",
                    "valueType": "string",
                    "value": self.driver_topic
                },
                {
                    "idShort": "operations",
                    "modelType": "SubmodelElementCollection",
                    "value": self.operations.iter().map(|o| o.to_basyx_json()).collect::<Vec<_>>()
                }
            ]
        })
    }
}