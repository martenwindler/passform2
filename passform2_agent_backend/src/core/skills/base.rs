use rclrs; // Rust ROS 2 Client Library
use std::sync::{Arc, Mutex};
use std::path::PathBuf;
use std::time::Duration;

// Lifecycle & Action Imports (angenommen rclrs unterstützt diese Schnittstellen)
use rclrs_lifecycle::{LifecycleNode, LifecycleState, TransitionResult};
use rclrs_action::{ActionServer, GoalHandle, GoalResponse, CancelResponse};

// Eigene Module
use crate::relay::ActionRelay;
use crate::mixins::SkillRequestMixin;

// Nachrichten (generiert via rosidl)
use passform_msgs::action::Passform;
use passform_msgs::msg::{Task, Cost};

/// Basis-Klasse für alle Single-Skills (Primitive)
pub struct PrimitiveLifecycle {
    node: Arc<rclrs::Node>,
    // Parameter & State
    skill_file: String,
    uuid: String,
    hardware_id: Option<String>,
    
    // Interne Komponenten
    basyx_client: Option<crate::basyx::BaSyxClient>, 
    goal_handle: Arc<Mutex<Option<GoalHandle<Passform>>>>,
    
    // Interfaces
    action_server: Option<ActionServer<Passform>>,
    request_subscription: Option<Arc<rclrs::Subscription<Task>>>,
    cost_publisher: Option<Arc<rclrs::Publisher<Cost>>>,
    
    // TF2 (via rust-tf2 oder ähnlichem)
    tf_buffer: Arc<Mutex<crate::util::TfBuffer>>,
}

impl PrimitiveLifecycle {
    pub fn new(node_name: &str) -> Self {
        let node = rclrs::Node::new(node_name).unwrap();
        
        // Parameter deklarieren (Rust-Syntax für Parameter-Handling)
        let skill_file = node.declare_parameter("skill_file").get_string();
        let uuid = node.declare_parameter("uuid").get_string();

        Self {
            node,
            skill_file,
            uuid,
            hardware_id: None,
            basyx_client: None,
            goal_handle: Arc::new(Mutex::new(None)),
            action_server: None,
            request_subscription: None,
            cost_publisher: None,
            tf_buffer: Arc::new(Mutex::new(crate::util::TfBuffer::new(Duration::from_secs(10)))),
        }
    }

    /// Hilfsmethode: Prüfen ob Task erreichbar
    fn is_task_reachable(&self, task: &Task) -> bool {
        let locations = vec![&task.condition.start, &task.condition.end]; // + waypoints
        locations.iter().all(|loc| self.is_reachable(loc))
    }

    pub fn is_reachable(&self, _location: &passform_msgs::msg::Location) -> bool {
        true // Override in spezifischen Skills
    }

    /// Berechnung der Kosten (entspricht _calculate_cost)
    fn calculate_cost(&self, task: &Task) -> Cost {
        let mut cost = Cost::default();
        cost.duration = self.estimate_duration(task);
        cost.earliest_start = self.node.get_clock().now().to_msg();
        cost.request_id = task.request_id.clone();
        // Topic Pfad normalisieren
        cost.action_topic = format!("{}/{}", self.node.get_namespace(), "skill_id").replace("//", "/");
        cost
    }

    pub fn estimate_duration(&self, _task: &Task) -> Duration {
        panic!("estimate_duration must be implemented by child");
    }
}

/// Implementierung der Lifecycle-Schnittstelle
impl LifecycleNode for PrimitiveLifecycle {
    fn on_configure(&mut self) -> TransitionResult {
        println!("Configuring Rust Primitive Skill...");
        
        // 1. BaSyx Initialisierung
        // Hier nutzen wir den Client aus dem vorherigen Schritt
        self.basyx_client = Some(crate::basyx::BaSyxClient::new("localhost", 8081, 8082, 3));
        
        // 2. Interfaces erstellen
        // (ActionServer Setup erfolgt hier...)

        TransitionResult::Success
    }

    fn on_activate(&mut self) -> TransitionResult {
        println!("Skill Active!");
        TransitionResult::Success
    }

    fn on_cleanup(&mut self) -> TransitionResult {
        if let Some(ref client) = self.basyx_client {
            let _ = client.remove_aas(&self.uuid);
        }
        TransitionResult::Success
    }
}

/// Zusammengesetzte Skills (Orchestrierung)
pub struct CompositeLifecycle {
    base: PrimitiveLifecycle,
    primitives: Vec<crate::util::Skill>,
    relay: Option<ActionRelay>,
}

impl CompositeLifecycle {
    pub fn new(node_name: &str) -> Self {
        Self {
            base: PrimitiveLifecycle::new(node_name),
            primitives: Vec::new(),
            relay: None,
        }
    }

    /// Orchestrierung: Führt Sub-Skills nacheinander aus
    pub fn execute_skill(&mut self, goal_handle: GoalHandle<Passform>) -> Passform::Result {
        let mut last_result = Passform::Result::default();
        
        for primitive in &self.primitives {
            if let Some(ref mut relay) = self.relay {
                relay.relay_goal(&goal_handle, &primitive.id_short);
                
                // Warte-Schleife (entspricht while rclpy.ok() ...)
                while relay.is_active() {
                    if goal_handle.is_cancel_requested() {
                        relay.cancel();
                        return Passform::Result::default();
                    }
                    std::thread::sleep(Duration::from_millis(100));
                }
                last_result = relay.get_result();
            }
        }
        
        goal_handle.succeed();
        last_result
    }
}