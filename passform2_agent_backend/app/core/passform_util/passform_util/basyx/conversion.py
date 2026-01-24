use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::path::Path;
use std::fs;
use std::collections::HashMap;

/// Repräsentiert die ModelingKind (Instance vs Template)
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum ModelingKind {
    #[serde(rename = "Instance")]
    Instance,
    #[serde(rename = "Template")]
    Template,
}

/// Die zentralen BaSyx Submodel-Elemente
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "modelType")]
pub enum SubmodelElement {
    #[serde(rename = "Property")]
    Property {
        #[serde(rename = "idShort")]
        id_short: String,
        #[serde(rename = "valueType")]
        value_type: String,
        value: Option<Value>,
        #[serde(skip_serializing_if = "Option::is_none")]
        qualifier: Option<Vec<Qualifier>>,
        kind: ModelingKind,
    },
    #[serde(rename = "SubmodelElementCollection")]
    Collection {
        #[serde(rename = "idShort")]
        id_short: String,
        value: Vec<SubmodelElement>,
        kind: ModelingKind,
    },
    #[serde(rename = "Blob")]
    Blob {
        #[serde(rename = "idShort")]
        id_short: String,
        #[serde(rename = "mimeType")]
        mime_type: String,
        value: String, // Base64 kodiert
    },
    #[serde(rename = "Range")]
    Range {
        #[serde(rename = "idShort")]
        id_short: String,
        #[serde(rename = "valueType")]
        value_type: String,
        min: Value,
        max: Value,
        kind: ModelingKind,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Qualifier {
    #[serde(rename = "type")]
    pub type_: String,
    #[serde(rename = "valueType")]
    pub value_type: String,
    pub value: String,
}

/// Verwandelt eine Datei in einen BaSyx Blob
pub fn data_to_blob(data_path: &Path, id_short: Option<String>) -> Result<SubmodelElement, std::io::Error> {
    let file_name = data_path.file_stem().unwrap().to_str().unwrap();
    let id = id_short.unwrap_or_else(|| file_name.to_string());
    
    // Einfaches Mime-Type Guessing (Rust hat keine Standard-Lib dafür, 
    // hier vereinfacht oder über crate 'mime_guess')
    let mime_type = match data_path.extension().and_then(|s| s.to_str()) {
        Some("png") => "image/png",
        Some("jpg") | Some("jpeg") => "image/jpeg",
        Some("pdf") => "application/pdf",
        _ => "application/octet-stream",
    };

    let data = fs::read(data_path)?;
    // BaSyx erwartet Blobs oft als Base64
    let base64_value = base64::encode(&data);

    Ok(SubmodelElement::Blob {
        id_short: id,
        mime_type: mime_type.to_string(),
        value: base64_value,
    })
}

/// Hilfsfunktion für die ROS-Typen-Konvertierung
fn get_basyx_datatype(ros_type: &str) -> &str {
    match ros_type {
        "int8" | "int16" => "Short",
        "int32" | "int64" => "Integer",
        "uint8" | "uint16" => "UnsignedShort",
        "uint32" | "uint64" => "UnsignedInt",
        "string" => "String",
        "double" | "float64" => "Double",
        "float" | "float32" => "Float",
        "boolean" | "bool" => "Boolean",
        "octet" | "byte" => "Byte",
        _ => "String",
    }
}

/// Fügt den ROS-Qualifier hinzu (identisch zur Python Logik)
fn add_ros_qualifier(data_type: &str) -> Vec<Qualifier> {
    vec![Qualifier {
        type_: "ROS".to_string(),
        value_type: "String".to_string(),
        value: data_type.to_string(),
    }]
}

/// Konvertiert (theoretische) ROS-Datenstrukturen in BaSyx-Elemente
/// In Rust würden diese Daten meist über rclrs oder serde_json reinkommen.
pub fn ros_to_dataelement(
    id_short: String, 
    data_type: &str, 
    value: Option<Value>, 
    kind: ModelingKind
) -> SubmodelElement {
    
    let mut clean_type = data_type.to_string();
    let mut is_array = false;

    if clean_type.contains("sequence<") || clean_type.contains("[]") {
        clean_type = clean_type.replace("sequence<", "").replace(">", "").replace("[]", "");
        is_array = true;
    }

    let final_id = if is_array { format!("{}_ARRAY", id_short) } else { id_short };

    // Komplexe ROS-Typen (pkg/Msg) behandeln wir als Collections
    if clean_type.contains('/') {
        // Hinweis: Echte Rekursion erfordert Zugriff auf ROS-Message-Definitionen.
        // Hier als Platzhalter für eine Collection:
        SubmodelElement::Collection {
            id_short: final_id,
            value: vec![], // Hier müssten die Child-Elemente rein
            kind,
        }
    } else {
        // Einfacher Typ
        SubmodelElement::Property {
            id_short: final_id,
            value_type: get_basyx_datatype(&clean_type).to_string(),
            value,
            qualifier: Some(add_ros_qualifier(&clean_type)),
            kind,
        }
    }
}

/// Erzeugt eine OperationVariable (ModelingKind: Template)
pub fn to_operation_variable(id_short: String, data_type: &str) -> SubmodelElement {
    ros_to_dataelement(id_short, data_type, None, ModelingKind::Template)
}