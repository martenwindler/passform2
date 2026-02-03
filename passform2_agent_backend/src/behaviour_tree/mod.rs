pub mod htn;
pub mod skill_node;
pub mod skills;
pub mod sequence_node;

pub use htn::{HTNPlanner, WorldState, Operator};
pub use skill_node::SkillNode;
pub use sequence_node::SequenceNode;
pub use skills::{SkillLibrary, SkillMetadata};

use async_trait::async_trait;

#[derive(Debug, PartialEq, Clone, Copy, serde::Serialize)]
pub enum NodeStatus {
    Idle,
    Success,
    Failure,
    Running,
}

#[async_trait]
pub trait BehaviorNode: Send + Sync {
    fn name(&self) -> String;
    async fn tick(&mut self) -> NodeStatus;
    // Hilfreich für das Frontend, um den aktuellen Status abzufragen ohne zu ticken
    fn get_status(&self) -> NodeStatus; 
}

/// Die "Fabrik"-Funktion: Erstellt rekursiv einen ausführbaren Baum aus Metadaten.
pub fn build_tree(skill: &SkillMetadata, library: &SkillLibrary) -> Box<dyn BehaviorNode> {
    // Falls der Skill 'primitives' hat, bauen wir eine Sequenz
    if let Some(ref primitives) = skill.primitives {
        let mut children = Vec::new();
        
        for primitive in primitives {
            // Suche nach dem passenden Skill-Typ in der Library
            if let Some(child_meta) = library.skills.iter().find(|s| s.skill_type == primitive.skill_type) {
                children.push(build_tree(child_meta, library));
            } else {
                tracing::warn!(
                    "⚠️ Unter-Skill vom Typ '{}' für Sequenz '{}' nicht in Library gefunden!", 
                    primitive.skill_type, skill.name
                );
            }
        }
        Box::new(SequenceNode::new(skill.name.clone(), children))
    } else {
        // Basis-Fall: Ein einfacher Blatt-Knoten (Action/Primitive)
        Box::new(SkillNode::new(skill.clone()))
    }
}