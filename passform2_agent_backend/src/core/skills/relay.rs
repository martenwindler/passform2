use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use rclrs;
use rclrs_action::{ActionClient, ClientGoalHandle, ServerGoalHandle};
use passform_agent_resources::action::Passform;

/// Das ActionRelay leitet Ziele von einem lokalen ActionServer 
/// an externe ActionServer (Treiber) weiter.
pub struct ActionRelay {
    node: Arc<rclrs::Node>,
    // Speichert ActionClients unter ihrem Key (id_short)
    relay_clients: HashMap<String, Arc<ActionClient<Passform>>>,
    
    // Handles für das aktuell aktive Ziel
    active_client_handle: Arc<Mutex<Option<ClientGoalHandle<Passform>>>>,
    active_server_handle: Arc<Mutex<Option<ServerGoalHandle<Passform>>>>,
    
    // Speicher für das letzte Ergebnis
    last_result: Arc<Mutex<Option<Passform::Result>>>,
}

impl ActionRelay {
    pub fn new(node: Arc<rclrs::Node>) -> Self {
        Self {
            node,
            relay_clients: HashMap::new(),
            active_client_handle: Arc::new(Mutex::new(None)),
            active_server_handle: Arc::new(Mutex::new(None)),
            last_result: Arc::new(Mutex::new(None)),
        }
    }

    /// Erstellt einen neuen ActionClient für einen Treiber
    pub fn create_action_client(&mut self, key: &str, action_name: &str) {
        let client = ActionClient::<Passform>::new(&self.node, action_name).unwrap();
        self.relay_clients.insert(key.to_string(), Arc::new(client));
    }

    /// Leitet ein Ziel (Goal) an den spezifischen Treiber-Server weiter
    pub fn relay_goal(&self, server_handle: ServerGoalHandle<Passform>, server_key: &str) {
        let client = match self.relay_clients.get(server_key) {
            Some(c) => c.clone(),
            None => {
                println!("Error: ActionClient for key {} not found", server_key);
                return;
            }
        };

        println!("Relaying goal to {}", server_key);

        // Referenzen für die Closures klonen
        let server_handle_clone = Arc::new(Mutex::new(server_handle.clone()));
        let active_client_handle = self.active_client_handle.clone();
        let last_result = self.last_result.clone();
        let active_server_handle = self.active_server_handle.clone();

        // 1. Goal an Treiber senden
        let goal = server_handle.get_goal();
        
        // Feedback-Callback: Reicht Feedback vom Treiber an den ursprünglichen Client weiter
        let feedback_callback = {
            let s_handle = server_handle_clone.clone();
            move |feedback: Passform::Feedback| {
                let mut s = s_handle.lock().unwrap();
                s.publish_feedback(feedback);
            }
        };

        // Ziel asynchron senden
        client.send_goal(goal, feedback_callback, move |goal_handle| {
            // Goal Response Callback
            if !goal_handle.is_accepted() {
                println!("Goal rejected by driver.");
                return;
            }
            
            println!("Goal accepted by driver.");
            *active_client_handle.lock().unwrap() = Some(goal_handle.clone());
            *active_server_handle.lock().unwrap() = Some(server_handle_clone.lock().unwrap().clone());

            // 2. Auf Resultat vom Treiber warten
            let res_client_handle = goal_handle.clone();
            let res_server_handle = server_handle_clone.clone();
            let res_result_store = last_result.clone();
            let res_active_server = active_server_handle.clone();

            std::thread::spawn(move || {
                let result = res_client_handle.get_result();
                
                // Resultat speichern und lokales Goal abschließen
                *res_result_store.lock().unwrap() = Some(result.result.clone());
                *res_active_server.lock().unwrap() = None;
                
                let mut s = res_server_handle.lock().unwrap();
                s.succeed(result.result);
            });
        });
    }

    pub fn is_active(&self) -> bool {
        self.active_server_handle.lock().unwrap().is_some()
    }

    pub fn cancel(&self) {
        let mut client_handle = self.active_client_handle.lock().unwrap();
        if let Some(handle) = client_handle.as_ref() {
            handle.cancel_goal();
            println!("Canceled goal on driver side.");
        }
        *client_handle = None;
    }

    pub fn get_result(&self) -> Passform::Result {
        self.last_result.lock().unwrap().clone().unwrap_or_default()
    }
}