use crate::behaviour_tree::{BehaviorNode, NodeStatus};
use crate::behaviour_tree::skills::SkillMetadata;
use async_trait::async_trait;
use tracing::info;

pub struct SkillNode {
    pub metadata: SkillMetadata,
    pub status: NodeStatus, // NEU: Speichert den aktuellen Zustand
}

impl SkillNode {
    pub fn new(metadata: SkillMetadata) -> Self {
        Self { 
            metadata,
            status: NodeStatus::Idle, // Startet immer im Idle-Modus
        }
    }
}

#[async_trait]
impl BehaviorNode for SkillNode {
    fn name(&self) -> String {
        self.metadata.name.clone()
    }

    // NEU: Implementierung für den Trait-Abgleich
    fn get_status(&self) -> NodeStatus {
        self.status
    }

    async fn tick(&mut self) -> NodeStatus {
        self.status = NodeStatus::Running;
        info!("🤖 BT Tick -> Skill: {} [{}]", self.metadata.name, self.metadata.skill_type);
        
        // Hier würde später die echte ROS-Logik stehen.
        // Aktuell setzen wir es direkt auf Success.
        self.status = NodeStatus::Success;
        self.status
    }
}