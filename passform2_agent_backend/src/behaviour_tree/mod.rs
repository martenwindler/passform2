pub mod skill_node;
pub mod skills;
pub mod sequence_node; // Neu hinzugefügt

// Re-Exports für bequemeren Zugriff in main.rs
pub use skill_node::SkillNode;
pub use sequence_node::SequenceNode;
pub use skills::{SkillLibrary, SkillMetadata};

use async_trait::async_trait;

#[derive(Debug, PartialEq, Clone, Copy, serde::Serialize)]
pub enum NodeStatus {
    Success,
    Failure,
    Running,
}

#[async_trait]
pub trait BehaviorNode: Send + Sync {
    fn name(&self) -> String;
    async fn tick(&mut self) -> NodeStatus;
}

/// Die "Fabrik"-Funktion: Erstellt rekursiv einen ausführbaren Baum aus Metadaten.
pub fn build_tree(skill: &SkillMetadata, library: &SkillLibrary) -> Box<dyn BehaviorNode> {
    // Falls der Skill 'primitives' hat, bauen wir eine Sequenz (z.B. Pick-and-Place)
    if let Some(ref primitives) = skill.primitives {
        let mut children = Vec::new();
        
        for primitive in primitives {
            // Wir suchen in der Library nach dem Skill, dessen TYP (z.B. "Pick") 
            // mit dem Typ im Primitive-Eintrag übereinstimmt.
            if let Some(child_meta) = library.skills.iter().find(|s| s.skill_type == primitive.skill_type) {
                // Rekursiver Aufruf: Ein CUSTOM Skill könnte theoretisch andere CUSTOM Skills enthalten
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
        // Basis-Fall: Es ist ein einfacher Skill (Blatt-Knoten)
        Box::new(SkillNode::new(skill.clone()))
    }
}