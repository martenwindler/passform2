use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use tokio::sync::RwLock;
use tracing::{info, warn, error, debug};
use rclrs;

use crate::core::types::{SkillType, Status};
use crate::core::types::plant_model::PlantModel;
use crate::managers::agent_manager::AgentManager;
use crate::managers::resource_manager::ResourceManager;

use passform_agent_resources::msg::PlanTask;
use passform_agent_resources::msg::PlanTaskCost;

pub struct SkillActionManager {
    pub _node: rclrs::Node,
    pub plant: Arc<RwLock<PlantModel>>,
    pub agent_manager: Arc<AgentManager>,
    pub resource_manager: Arc<ResourceManager>,
    task_requests: Arc<Mutex<BTreeMap<String, Option<PlanTaskCost>>>>,
}

impl SkillActionManager {
    pub fn new(
        node: rclrs::Node,
        plant: Arc<RwLock<PlantModel>>,
        agent_manager: Arc<AgentManager>,
        resource_manager: Arc<ResourceManager>,
    ) -> Self {
        Self {
            _node: node,
            plant,
            agent_manager,
            resource_manager,
            task_requests: Arc::new(Mutex::new(BTreeMap::new())),
        }
    }

    /// Prüft, ob die physischen Voraussetzungen (Versorgung, Position) erfüllt sind
    pub async fn validate_preconditions(&self, agent_id: &str, skill: SkillType) -> Result<(), String> {
        // 1. Agenten-Daten holen
        let agents_guard = self.agent_manager.agents.read().await;
        let agent = agents_guard.get(agent_id)
            .ok_or_else(|| format!("Agent {} nicht im System registriert", agent_id))?;

        // 2. Prüfen, ob der Agent in einer Bay steht
        let plant_guard = self.plant.read().await;
        let current_bay = plant_guard.bays.iter()
            .find(|b| b.occupation && b.module_uuid == agent_id)
            .ok_or_else(|| format!("Validierungsfehler: Agent {} befindet sich an keiner gültigen Bay-Position", agent_id))?;

        // 3. Modul-Specs prüfen (falls ein Modul an der Bay registriert ist)
        // Hier nutzen wir die module_uuid, um die Hardware-Fähigkeiten abzugleichen
        match skill {
            SkillType::Grasp | SkillType::Pressure => {
                // Beispiel: Braucht Vakuum. Wir könnten hier das tatsächliche Modul aus dem ResourceManager laden
                info!("🔍 Check: Benötigt {} Vakuum-Versorgung in {}?", agent_id, current_bay.name);
                
                if current_bay.name.contains("Human") {
                    return Err("Sicherheits-Interlock: Vakuum-Operationen in Human-Bays sind untersagt!".to_string());
                }
            },
            SkillType::MoveBody | SkillType::Move => {
                if agent.status == Status::Error {
                    return Err(format!("Abbruch: Agent {} ist im Fehlerzustand", agent_id));
                }
            },
            _ => debug!("Skill {:?} erfordert keine spezielle Ressourcenprüfung", skill),
        }

        Ok(())
    }

    /// Führt einen Skill aus, nachdem die Bedingungen geprüft wurden
    pub async fn execute_skill(&self, agent_id: String, skill: SkillType) -> Result<(), String> {
        // Erst die "Hardware-Polizei" fragen
        self.validate_preconditions(&agent_id, skill).await?;

        info!("🚀 Preconditions OK. Starte Skill {:?} für {}", skill, agent_id);
        
        // Hier würde nun der tatsächliche ROS Action Client Aufruf folgen
        // Da wir momentan noch auf Stubs setzen:
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
        
        Ok(())
    }

    // --- Bestands-Methoden ---

    pub fn register_driver_client(&mut self, _agent_id: &str, _topic: &str) {
        info!("Action-Client Mapping registriert für: {}", _agent_id);
    }

    pub async fn abort_current_action(&self) {
        warn!("Not-Halt: Alle laufenden Skills werden abgebrochen.");
    }

    pub async fn request_cost(&self, _task: PlanTask, _skill_types: Vec<i32>, _topic: &str) -> PlanTaskCost {
        // Hier könnte man später reale Fahrzeiten/Kosten berechnen
        PlanTaskCost::default()
    }
}

pub struct PrimitiveLifecycle {
    pub node: Arc<rclrs::Node>,
    pub uuid: String,
}

impl PrimitiveLifecycle {
    pub fn new(node: Arc<rclrs::Node>) -> Self {
        let uuid_param = node.declare_parameter::<Arc<str>>("uuid")
            .default(Arc::from("default_primitive"))
            .mandatory()
            .expect("UUID Parameter Fehler");

        let uuid = uuid_param.get().to_string();
        Self { node, uuid }
    }
}