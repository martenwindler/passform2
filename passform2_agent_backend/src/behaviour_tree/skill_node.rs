use crate::behaviour_tree::{BehaviorNode, NodeStatus};
use crate::behaviour_tree::skills::SkillMetadata;
use async_trait::async_trait;
use tracing::info;

pub struct SkillNode {
    pub metadata: SkillMetadata,
}

impl SkillNode {
    // Das 'pub' hier ist entscheidend, damit main.rs die Funktion sieht!
    pub fn new(metadata: SkillMetadata) -> Self {
        Self { metadata }
    }
}

#[async_trait]
impl BehaviorNode for SkillNode {
    fn name(&self) -> String {
        self.metadata.name.clone()
    }

    async fn tick(&mut self) -> NodeStatus {
        info!("🤖 BT Tick -> Skill: {} [{}]", self.metadata.name, self.metadata.skill_type);
        NodeStatus::Success
    }
}