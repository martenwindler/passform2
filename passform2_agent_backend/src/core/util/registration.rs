use std::sync::Arc;
use rclrs;

pub fn register_node(node: &Arc<rclrs::Node>) {
    // In Jazzy rclrs liefert declare_parameter einen Builder.
    // Wir schließen ihn mit .mandatory() ab, um ein MandatoryParameter-Objekt zu erhalten.
    let uuid_param = node.declare_parameter::<Arc<str>>("uuid")
        .default(Arc::from("unknown_agent"))
        .mandatory() // Das ist die Methode, die den Builder konsumiert!
        .expect("Fehler beim Deklarieren des UUID-Parameters");

    // MandatoryParameter hat eine .get() Methode (siehe Zeile 484 im Quellcode)
    let uuid = uuid_param.get().to_string();

    tracing::info!("Node registriert mit UUID: {}", uuid);
}