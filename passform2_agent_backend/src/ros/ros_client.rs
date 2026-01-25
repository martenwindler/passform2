use std::sync::Arc;
use rclrs::{Context, Node, Executor, SpinOptions, CreateBasicExecutor};
use crate::managers::AgentManager;
use passform_agent_resources::msg::{AgentInfo, PathRequest};
use tracing::{info, error};

pub struct RosClient {
    // Wir speichern den Node direkt als Struct, wie in deiner lib.rs definiert
    pub node: Node,
}

impl RosClient {
    /// Erstellt einen neuen RosClient. 
    /// Gibt den Client und den Executor zurück, da der Executor für das Spinning benötigt wird.
    pub fn new(context: &Context) -> Result<(Self, Executor), rclrs::RclrsError> {
        // 1. Nutzt den Trait 'CreateBasicExecutor' aus deiner lib.rs
        let mut executor = context.create_basic_executor();
        
        // 2. Erstellt den Node über den Executor (genau wie im Basic Usage Beispiel deines Codes)
        let node = executor.create_node("passform_agent_backend")?;
        
        Ok((Self { node }, executor))
    }

    pub fn publish_path_request(&self, _req: PathRequest) -> Result<(), rclrs::RclrsError> {
        info!("Sende PathRequest über ROS...");
        Ok(())
    }

    pub async fn run(&self, mut executor: Executor, agent_manager: Arc<AgentManager>) -> Result<(), rclrs::RclrsError> {
        let am_info = Arc::clone(&agent_manager);
        
        // Subscription erstellen (Node-Methoden sind jetzt direkt verfügbar)
        let _agent_info_sub = self.node.create_subscription::<AgentInfo, _>(
            "agent_info",
            move |msg: AgentInfo| {
                let am = Arc::clone(&am_info);
                tokio::spawn(async move {
                    am.sync_from_ros(
                        msg.agent_id,
                        msg.module_type,
                        msg.position.x,
                        msg.position.y,
                        msg.orientation as i32,
                    ).await;
                });
            },
        )?;

        info!("🚀 ROS 2 Jazzy Backend aktiv.");

        // 3. Das Spinning: Wir bewegen den Executor in einen blockierenden Thread
        tokio::task::spawn_blocking(move || {
            // Nutzt die spin-Methode aus deiner executor.rs:53
            executor.spin(SpinOptions::default());
        });

        Ok(())
    }
}