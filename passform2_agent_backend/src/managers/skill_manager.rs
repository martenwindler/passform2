use std::collections::HashMap;
use std::sync::Arc;
use serde::{Deserialize, Serialize};
use tokio::sync::RwLock;
use tracing::{info, error, warn};
use socketioxide::SocketIo;

// --- DOMAIN MODELS ---

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Skill {
    pub id: String,
    pub name: String,
    pub description: String,
    pub complexity: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SkillExecutionRequest {
    pub agent_id: String,
    pub skill_id: String,
    pub parameters: serde_json::Value,
}

#[derive(Serialize, Clone, Debug)]
pub struct SkillStatusUpdate {
    pub agent_id: String,
    pub skill_id: String,
    pub status: String, // "running", "completed", "failed"
    pub progress: i32,    // 0-100
}

// --- MANAGER ---

pub struct SkillManager {
    // Katalog aller verfügbaren Skills im System (aus SSoT geladen)
    pub registry: RwLock<HashMap<String, Skill>>,
    // Tracking der laufenden Skills
    pub active_executions: RwLock<HashMap<String, SkillStatusUpdate>>,
}

impl SkillManager {
    pub fn new() -> Self {
        info!("✅ SkillManager: Zentrale Skill-Registry bereit.");
        Self {
            registry: RwLock::new(HashMap::new()),
            active_executions: RwLock::new(HashMap::new()),
        }
    }

    /// Lädt die Skill-Definitionen (z.B. aus der config.json)
    pub async fn load_skills_from_ssot(&self, skills_data: Vec<Skill>) {
        let mut registry = self.registry.write().await;
        registry.clear();
        for skill in skills_data {
            registry.insert(skill.id.clone(), skill);
        }
        info!("📚 {} Skills in die Registry geladen.", registry.len());
    }

    /// Triggert die Ausführung eines Skills über ROS
    pub async fn trigger_skill(
        &self, 
        req: SkillExecutionRequest, 
        _ros_client: Arc<crate::ros::ros_client::RosClient>,
        io: &SocketIo
    ) {
        let registry = self.registry.read().await;
        
        if !registry.contains_key(&req.skill_id) {
            error!("❌ Skill '{}' existiert nicht in der Registry!", req.skill_id);
            return;
        }

        info!("⚡ Starte Skill '{}' für Agent '{}'", req.skill_id, req.agent_id);

        // Hier würde der rclrs-Publisher eine Nachricht absetzen
        // ros_client.publish_skill_trigger(req.clone());

        // Initialen Status ans Frontend senden
        let initial_status = SkillStatusUpdate {
            agent_id: req.agent_id.clone(),
            skill_id: req.skill_id.clone(),
            status: "running".to_string(),
            progress: 0,
        };

        self.update_execution_status(initial_status, io).await;
    }

    /// Callback-Handler für Status-Updates von ROS
    pub async fn update_execution_status(&self, update: SkillStatusUpdate, io: &SocketIo) {
        let mut active = self.active_executions.write().await;
        
        info!("🔄 Skill-Update: Agent {} -> {} ({}%)", 
            update.agent_id, update.status, update.progress);

        // In der Registry speichern
        active.insert(update.agent_id.clone(), update.clone());

        // Direkt per WebSocket an das Elm-Frontend pushen
        io.emit("skill_status_update", &update).ok();

        // Wenn fertig, aus aktiven Executions entfernen (Cleanup)
        if update.status == "completed" || update.status == "failed" {
            active.remove(&update.agent_id);
        }
    }
}