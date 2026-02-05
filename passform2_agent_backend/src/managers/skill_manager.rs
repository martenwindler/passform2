use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use tracing::info;
use rclrs;

use passform_agent_resources::msg::PlanTask;
use passform_agent_resources::msg::PlanTaskCost;

pub struct SkillActionManager {
    node: rclrs::Node,
    task_requests: Arc<Mutex<BTreeMap<String, Option<PlanTaskCost>>>>,
}

impl SkillActionManager {
    pub fn new(node: rclrs::Node) -> Self {
        Self {
            node,
            task_requests: Arc::new(Mutex::new(BTreeMap::new())),
        }
    }

    pub fn register_driver_client(&mut self, _agent_id: &str, _topic: &str) {
        info!("Action-Client deaktiviert (Stub).");
    }

    pub async fn execute_action_relay(&self, _handle: String, _agent_id: &str) { }

    pub async fn get_last_result(&self) -> String {
        "Disabled".to_string()
    }

    pub async fn abort_current_action(&self) { }

    pub async fn request_cost(&self, _task: PlanTask, _skill_types: Vec<i32>, _topic: &str) -> PlanTaskCost {
        PlanTaskCost::default()
    }
}

pub struct PrimitiveLifecycle {
    pub node: Arc<rclrs::Node>,
    pub uuid: String,
}

impl PrimitiveLifecycle {
    pub fn new(node: Arc<rclrs::Node>) -> Self {
        // Deklarieren und direkt konsumieren
        let uuid_param = node.declare_parameter::<Arc<str>>("uuid")
            .default(Arc::from("default_primitive"))
            .mandatory()
            .expect("UUID Parameter Fehler");

        let uuid = uuid_param.get().to_string();
            
        Self { node, uuid }
    }
}