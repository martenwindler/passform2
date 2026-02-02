use std::collections::HashMap;
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use futures::channel::oneshot;

use rclrs;
use rclrs::{Node, ServiceIDL}; 
use passform_msgs::srv::Discover; 

pub struct DiscoverPassform {
    pub node: Arc<Node>,
    discovery_performed: bool,
    base_services: HashMap<String, String>,
    basyx_config: HashMap<String, String>,
}

impl DiscoverPassform {
    pub fn new(node: Arc<Node>) -> Self {
        Self {
            node,
            discovery_performed: false,
            base_services: HashMap::new(),
            basyx_config: HashMap::new(),
        }
    }

    pub fn discover(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let discover_topic = "/passform/discover";
        let client = self.node.create_client::<Discover>(discover_topic)?;
        let context = self.node.commands().context(); 

        // Warten auf den Service
        while !client.service_is_ready()? {
            if !context.ok() { return Err("Interrupted".into()); }
            println!("Warte auf Discovery Service '{}'...", discover_topic);
            thread::sleep(Duration::from_millis(500));
        }

        // Channel für die Antwort
        let (tx, mut rx) = oneshot::channel::<<Discover as ServiceIDL>::Response>();
        let request = <Discover as ServiceIDL>::Request::default();
        
        // Service-Call absetzen
        // In Jazzy ist msg ein Option, und wir unterdrücken die Promise-Warnung
        let _promise = client.call_then(request, move |response, _info: rclrs::ServiceInfo| {
            let _ = tx.send(response);
        })?;

        println!("Sende Discovery Request...");
        
        // Polling auf die Antwort
        loop {
            if !context.ok() { return Err("Interrupted during call".into()); }
            
            match rx.try_recv() {
                Ok(Some(response)) => {
                    self.parse_response(response)?;
                    break;
                }
                Ok(None) => thread::sleep(Duration::from_millis(100)),
                Err(_) => return Err("Service response channel closed unexpectedly".into()),
            }
        }

        self.discovery_performed = true;
        Ok(())
    }

    fn parse_response(&mut self, resp: <Discover as ServiceIDL>::Response) -> Result<(), Box<dyn std::error::Error>> {
        if resp.server_description.is_empty() {
            return Err("Discovery erhalten, aber server_description ist leer.".into());
        }

        for ds in resp.server_description {
            let data: HashMap<String, String> = ds.values
                .into_iter()
                .map(|kv| (kv.key, kv.value))
                .collect();

            match ds.name.to_lowercase().as_str() {
                "services" => self.base_services = data,
                "basyx" => self.basyx_config = data,
                _ => {}
            }
        }
        Ok(())
    }
}