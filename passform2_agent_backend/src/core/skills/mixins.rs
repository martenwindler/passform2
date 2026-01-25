use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use uuid::Uuid;

use rclrs;
use rclrs::{Node, Subscription, Publisher, QOS_PROFILE_DEFAULT};

// ROS Messages
use passform_msgs::msg::{Task, Cost};

/// In Rust kapseln wir die "Mixin"-Logik in einem Manager, 
/// den der Node als Member besitzt.
pub struct SkillRequestManager {
    // OrderedDict Ersatz: BTreeMap hält die Reihenfolge der Keys (IDs)
    task_requests: Arc<Mutex<BTreeMap<String, Option<Cost>>>>,
}

impl SkillRequestManager {
    pub fn new() -> Self {
        Self {
            task_requests: Arc::new(Mutex::new(BTreeMap::new())),
        }
    }

    /// Callback für eintreffende Kosten-Antworten
    pub fn cost_callback(&self, msg: Cost) {
        let mut reqs = self.task_requests.lock().unwrap();
        if reqs.contains_key(&msg.request_id) {
            reqs.insert(msg.request_id.clone(), Some(msg));
        }
    }

    /// Berechnet die Gesamtkosten aus allen Teilantworten
    pub fn get_final_cost(&self) -> Result<Cost, String> {
        let reqs = self.task_requests.lock().unwrap();
        
        if reqs.is_empty() {
            return Err("No task requests found".to_string());
        }

        let mut final_cost = Cost::default();
        let mut first = true;

        for (_, cost_opt) in reqs.iter() {
            if let Some(cost) = cost_opt {
                if first {
                    final_cost.earliest_start = cost.earliest_start.clone();
                    first = false;
                } else {
                    // Min earliest start (vereinfacht, Logik wie im Python original)
                    // Hier müsste ein echter Zeitvergleich stattfinden
                }
                
                // Duration & Energy aufsummieren
                final_cost.energy += cost.energy;
                // final_cost.duration = sum_durations(...)
            }
        }

        Ok(final_cost)
    }

    /// Die Kernlogik zum Anfordern von Kosten (Blocking-Wait wie im Python Original)
    pub fn request_cost(
        &self, 
        node: &Arc<Node>, 
        task: Task, 
        required_skill_types: Vec<i32>,
        request_topic: &str,
    ) -> Cost {
        // Publisher erstellen
        let publisher = node.create_publisher::<Task>(request_topic, QOS_PROFILE_DEFAULT).unwrap();
        
        {
            let mut reqs = self.task_requests.lock().unwrap();
            reqs.clear();
            
            for skill_type in required_skill_types {
                let mut sub_task = task.clone();
                sub_task.request_id = Uuid::new_v4().to_string();
                sub_task.type_field.skill_type = skill_type;
                
                reqs.insert(sub_task.request_id.clone(), None);
                publisher.publish(&sub_task).unwrap();
            }
        }

        // Wait-Loop (0.2s Intervalle wie im Original)
        let start_wait = Instant::now();
        let timeout = Duration::from_secs(5); // Sicherheits-Timeout

        while Instant::now() - start_wait < timeout {
            let all_done = {
                let reqs = self.task_requests.lock().unwrap();
                reqs.values().all(|v| v.is_some())
            };

            if all_done { break; }
            std::thread::sleep(Duration::from_millis(200));
        }

        self.get_final_cost().unwrap_or_else(|_| {
            let mut error_cost = Cost::default();
            error_cost.request_id = "".to_string();
            error_cost
        })
    }
}