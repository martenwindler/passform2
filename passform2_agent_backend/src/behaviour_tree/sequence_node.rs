use crate::behaviour_tree::{BehaviorNode, NodeStatus};
use async_trait::async_trait;
use tracing::info;

pub struct SequenceNode {
    pub name: String,
    pub children: Vec<Box<dyn BehaviorNode>>,
    pub current_child_index: usize,
}

impl SequenceNode {
    pub fn new(name: String, children: Vec<Box<dyn BehaviorNode>>) -> Self {
        Self {
            name,
            children,
            current_child_index: 0,
        }
    }
}

#[async_trait]
impl BehaviorNode for SequenceNode {
    fn name(&self) -> String {
        self.name.clone()
    }

    async fn tick(&mut self) -> NodeStatus {
        if self.children.is_empty() {
            return NodeStatus::Success;
        }

        if self.current_child_index >= self.children.len() {
            self.current_child_index = 0; // Reset für nächsten Durchlauf
        }

        let child = &mut self.children[self.current_child_index];
        info!("🔀 Sequence '{}' tickt Kind: {}", self.name, child.name());
        
        match child.tick().await {
            NodeStatus::Success => {
                self.current_child_index += 1;
                if self.current_child_index >= self.children.len() {
                    self.current_child_index = 0;
                    NodeStatus::Success
                } else {
                    NodeStatus::Running
                }
            }
            NodeStatus::Failure => {
                self.current_child_index = 0;
                NodeStatus::Failure
            }
            NodeStatus::Running => NodeStatus::Running,
        }
    }
}