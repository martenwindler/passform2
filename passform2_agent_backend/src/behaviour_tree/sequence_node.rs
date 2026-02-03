use crate::behaviour_tree::{BehaviorNode, NodeStatus};
use async_trait::async_trait;
use tracing::info;

pub struct SequenceNode {
    pub name: String,
    pub children: Vec<Box<dyn BehaviorNode>>,
    pub current_child_index: usize,
    pub status: NodeStatus, // NEU: Eigenen Status tracken
}

impl SequenceNode {
    pub fn new(name: String, children: Vec<Box<dyn BehaviorNode>>) -> Self {
        Self {
            name,
            children,
            current_child_index: 0,
            status: NodeStatus::Idle, // Initial auf Idle
        }
    }
}

#[async_trait]
impl BehaviorNode for SequenceNode {
    fn name(&self) -> String {
        self.name.clone()
    }

    // Trait-Erfüllung
    fn get_status(&self) -> NodeStatus {
        self.status
    }

    async fn tick(&mut self) -> NodeStatus {
        // Wenn wir anfangen, setzen wir uns auf Running
        self.status = NodeStatus::Running;

        if self.children.is_empty() {
            self.status = NodeStatus::Success;
            return NodeStatus::Success;
        }

        // Falls wir nach einem Erfolg/Fehler erneut getickt werden
        if self.current_child_index >= self.children.len() {
            self.current_child_index = 0;
        }

        let child = &mut self.children[self.current_child_index];
        info!("🔀 Sequence '{}' tickt Kind: {} (Index: {})", 
            self.name, child.name(), self.current_child_index);
        
        match child.tick().await {
            NodeStatus::Success => {
                self.current_child_index += 1;
                if self.current_child_index >= self.children.len() {
                    // Alle Kinder erfolgreich durchlaufen
                    self.current_child_index = 0;
                    self.status = NodeStatus::Success;
                    NodeStatus::Success
                } else {
                    // Nächstes Kind ist dran, Sequenz läuft noch
                    self.status = NodeStatus::Running;
                    NodeStatus::Running
                }
            }
            NodeStatus::Failure => {
                // Ein Kind ist gescheitert -> gesamte Sequenz scheitert
                self.current_child_index = 0;
                self.status = NodeStatus::Failure;
                NodeStatus::Failure
            }
            NodeStatus::Running | NodeStatus::Idle => {
                // Kinderarbeit
                self.status = NodeStatus::Running;
                NodeStatus::Running
            }
        }
    }
}