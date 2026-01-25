#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "modelType")]
pub enum SubmodelElement {
    Property {
        #[serde(rename = "idShort")]
        id_short: String,
        #[serde(rename = "valueType")]
        value_type: String,
        value: String,
    },
    SubmodelElementCollection {
        #[serde(rename = "idShort")]
        id_short: String,
        value: Vec<SubmodelElement>,
    },
    // Weitere Typen wie Blob, Range etc.
}

pub fn ros_to_basyx_type(ros_type: &str) -> &'static str {
    match ros_type {
        "int32" => "Integer",
        "float64" | "double" => "Double",
        "string" => "String",
        "bool" | "boolean" => "Boolean",
        _ => "String",
    }
}