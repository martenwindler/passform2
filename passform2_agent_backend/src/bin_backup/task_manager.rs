use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use uuid::Uuid;

use rclrs;
use rclrs::{Node, Publisher, Subscription, QOS_PROFILE_DEFAULT};
use rclrs_action::{ActionServer, GoalResponse, CancelResponse, GoalHandle};

// Eigene Module/Komponenten
use crate::relay::ActionRelay;
use crate::mixins::SkillRequestManager;

// Nachrichten
use passform_agent_resources::msg::{Task, Cost};
use passform_agent_resources::action::Passform;
use std_msgs::msg::String as StringMsg;

pub struct TaskManager {
    node: Arc<Node>,
    // State
    active_id: Arc<Mutex<i32>>,
    available_skills: Arc<Mutex<Vec<Cost>>>,
    cost_search_active: Arc<Mutex<bool>>,
    request_msg: Arc<Mutex<Option<Task>>>,
    
    // Komponenten
    skill_manager: SkillRequestManager,
    relay: Arc<Mutex<Option<ActionRelay>>>,

    // Interfaces
    request_publisher: Arc<Publisher<Task>>,
    feedback_publisher: Arc<Publisher<StringMsg>>,
}

impl TaskManager {
    pub fn new(node_name: &str) -> Arc<Self> {
        let node = rclrs::Node::new(node_name).unwrap();
        
        // Parameter (Defaults wie im Python-Code)
        let request_topic = "/skill_request";
        let cost_topic = "/skill_response";
        let feedback_topic = "/task_feedback";

        let request_publisher = node.create_publisher::<Task>(request_topic, QOS_PROFILE_DEFAULT).unwrap();
        let feedback_publisher = node.create_publisher::<StringMsg>(feedback_topic, QOS_PROFILE_DEFAULT).unwrap();

        let manager = Arc::new(Self {
            node: node.clone(),
            active_id: Arc::new(Mutex::new(0)),
            available_skills: Arc::new(Mutex::new(Vec::new())),
            cost_search_active: Arc::new(Mutex::new(false)),
            request_msg: Arc::new(Mutex::new(None)),
            skill_manager: SkillRequestManager::new(),
            relay: Arc::new(Mutex::new(None)),
            request_publisher,
            feedback_publisher,
        });

        // Cost Listener
        let manager_clone = manager.clone();
        let _cost_listener = node.create_subscription::<Cost, _>(
            cost_topic,
            QOS_PROFILE_DEFAULT,
            move |msg: Cost| manager_clone.cost_callback(msg),
        ).unwrap();

        // Timer für Feedback (0.5s)
        let manager_clone_timer = manager.clone();
        let _feedback_timer = node.create_timer(Duration::from_millis(500), move || {
            manager_clone_timer.publish_feedback();
        });

        manager
    }

    fn cost_callback(&self, msg: Cost) {
        let search_active = self.cost_search_active.lock().unwrap();
        if !*search_active { return; }

        if self.response_is_valid(&msg) {
            let mut skills = self.available_skills.lock().unwrap();
            skills.push(msg);
        }
    }

    fn response_is_valid(&self, msg: &Cost) -> bool {
        let req_msg = self.request_msg.lock().unwrap();
        if let Some(ref req) = *req_msg {
            if msg.request_id != req.request_id { return false; }
            if msg.action_topic.is_empty() { return false; }
            println!("Received valid offer for skill {}", msg.skill_uuid);
            return true;
        }
        false
    }

    pub fn execute_skill(&self, goal_handle: GoalHandle<Passform>) -> Passform::Result {
        // 1. Kostenanfrage (Broadcasting)
        let task = goal_handle.get_goal().task.clone();
        let wait_time = 3.0; // response_wait_time Parameter

        {
            let mut skills = self.available_skills.lock().unwrap();
            skills.clear();
            let mut req_msg = self.request_msg.lock().unwrap();
            let mut task_copy = task.clone();
            task_copy.request_id = Uuid::new_v4().to_string();
            *req_msg = Some(task_copy.clone());
            
            *self.cost_search_active.lock().unwrap() = true;
            self.request_publisher.publish(&task_copy).unwrap();
        }

        // 2. Warten auf Antworten
        std::thread::sleep(Duration::from_secs_f32(wait_time));
        *self.cost_search_active.lock().unwrap() = false;

        // 3. Optimalen Skill wählen
        let winning_skill = {
            let skills = self.available_skills.lock().unwrap();
            if skills.is_empty() {
                goal_handle.abort();
                return Passform::Result::default();
            }
            skills[0].clone() // Einfaches Picking: Erstes Element (Todo: Kostenvergleich)
        };

        // 4. Relay initialisieren & Ziel weiterleiten
        let mut relay_guard = self.relay.lock().unwrap();
        let relay = ActionRelay::new(self.node.clone());
        relay.create_action_client(&winning_skill.skill_uuid, &winning_skill.action_topic);
        
        relay.relay_goal(goal_handle.clone(), &winning_skill.skill_uuid);
        *relay_guard = Some(relay);

        // Task ID extrahieren für Feedback
        for param in &task.optional_parameter {
            if param.name == "task_id" {
                *self.active_id.lock().unwrap() = param.value.integer_value;
            }
        }

        // 5. Überwachung der Ausführung
        while rclpy::ok() {
            let relay_active = {
                let r = self.relay.lock().unwrap();
                r.as_ref().map_or(false, |rel| rel.is_active())
            };

            if !relay_active { break; }

            if goal_handle.is_cancel_requested() {
                if let Some(ref r) = *self.relay.lock().unwrap() {
                    r.cancel();
                }
                return Passform::Result::default();
            }
            std::thread::sleep(Duration::from_millis(100));
        }

        // Erfolg setzen & Resultat zurückgeben
        self.set_success();
        let result = self.relay.lock().unwrap().as_ref().unwrap().get_result();
        goal_handle.succeed(result.clone());
        result
    }

    fn publish_feedback(&self) {
        // Logik für Feedback-Publisher
        let id = self.active_id.lock().unwrap();
        self.feedback_publisher.publish(&StringMsg { data: id.to_string() }).unwrap();
    }

    fn set_success(&self) {
        let mut id = self.active_id.lock().unwrap();
        *id += 1;
    }
}