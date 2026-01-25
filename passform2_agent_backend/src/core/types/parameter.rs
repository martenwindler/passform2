use serde_json::Value;
use glob::Pattern;

pub mod parameter {
    use super::*;

    /// Sucht nach dem ersten Vorkommen von `id_short` in einem Submodel oder einer Collection.
    /// Unterstützt Wildcards wie '*' und '?'.
    pub fn get_parameter<'a>(obj: &'a Value, id_short_pattern: &str) -> Option<&'a Value> {
        // Pattern für Wildcards vorbereiten
        let pattern = Pattern::new(id_short_pattern).ok()?;

        // Wir prüfen, ob das Objekt "submodelElements" (Submodel) oder "value" (Collection) hat
        let elements = if let Some(e) = obj.get("submodelElements").and_then(|v| v.as_array()) {
            e
        } else if let Some(v) = obj.get("value").and_then(|v| v.as_array()) {
            v
        } else {
            return None;
        };

        for element in elements {
            // Check das aktuelle Element
            if let Some(id) = element.get("idShort").and_then(|id| id.as_str()) {
                if pattern.matches(id) {
                    return Some(element);
                }
            }

            // Rekursive Suche in Collections (falls das Element wieder ein "value" Array hat)
            if element.get("value").is_array() {
                if let Some(found) = get_parameter(element, id_short_pattern) {
                    return Some(found);
                }
            }
        }

        None
    }

    /// Hilfsfunktion, um direkt den Wert (Value) eines Parameters zu bekommen
    pub fn get_parameter_value(obj: &Value, id_short: &str) -> Value {
        get_parameter(obj, id_short)
            .and_then(|p| p.get("value"))
            .cloned()
            .unwrap_or(Value::Null)
    }
}