use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::any::Any;
use rclrs::{Node, Publisher, SubscriptionOptions, RclrsError, Context, CreateBasicExecutor, SpinOptions, RclReturnCode};
use passform_msgs::msg::{AgentInfo, PathRequest}; 
use tracing::{info, error, warn};

pub struct RosClient {
    pub node: Node,
    pub subscriptions: Mutex<Vec<Arc<dyn Any + Send + Sync>>>,
    pub path_pub: Arc<Publisher<PathRequest>>,
    pub is_running: Arc<AtomicBool>,
    // NEU: Handle zur Tokio-Runtime speichern
    pub rt_handle: tokio::runtime::Handle,
}

impl RosClient {
    pub fn new(context: &Context) -> Result<Self, RclrsError> {
        // Capture den Handle der aktuellen Tokio-Runtime (wird in main aufgerufen)
        let rt_handle = tokio::runtime::Handle::current();
        
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let context_clone = context.clone();
        let is_running = Arc::new(AtomicBool::new(true));
        let is_running_thread = is_running.clone();

        // Hintergrund-Thread für ROS Executor
        std::thread::spawn(move || {
            let mut executor = context_clone.create_basic_executor();
            
            // Executor erstellt und besitzt den Node direkt
            let node = match executor.create_node("passform_agent_backend") {
                Ok(n) => n,
                Err(e) => {
                    error!("❌ Konnte ROS Node nicht erstellen: {:?}", e);
                    return;
                }
            };

            let _ = tx.send(node.clone());

            info!("🔄 ROS 2 Executor Loop gestartet...");

            while is_running_thread.load(Ordering::SeqCst) {
                // SpinOptions müssen in jedem Durchlauf neu erstellt werden (kein Copy-Trait)
                let mut spin_options = SpinOptions::default();
                spin_options.timeout = Some(std::time::Duration::from_millis(100));

                let _ = executor.spin(spin_options);
            }
            warn!("🛑 ROS 2 Executor Loop beendet.");
        });

        // Node sicher aus dem Thread empfangen
        let node = rx.recv().map_err(|_| RclrsError::RclError { 
            code: RclReturnCode::Error,
            msg: None 
        })?;

        let path_pub = node.create_publisher::<PathRequest>("path_requests")?;

        Ok(Self { 
            node,
            subscriptions: Mutex::new(Vec::new()),
            path_pub: Arc::new(path_pub),
            is_running,
            rt_handle,
        })
    }

    pub fn stop(&self) {
        self.is_running.store(false, Ordering::SeqCst);
    }

    pub fn publish_path_request(&self, req: PathRequest) -> Result<(), RclrsError> {
        self.path_pub.publish(&req)
    }

    pub async fn run(&self, agent_manager: Arc<crate::managers::AgentManager>) -> Result<(), RclrsError> {
        let am_clone = Arc::clone(&agent_manager);
        // Handle für den Callback klonen
        let rt = self.rt_handle.clone(); 
        
        let sub = self.node.create_subscription::<AgentInfo, _>(
            SubscriptionOptions::new("agent_info"),
            move |msg: AgentInfo| {
                info!("📥 ROS Nachricht empfangen: ID={}, Typ={}", msg.agent_id, msg.module_type);
                
                let am = Arc::clone(&am_clone);
                // WICHTIG: Nutze den Handle, um den Task in die Tokio-Runtime zu schieben
                rt.spawn(async move {
                    am.sync_from_ros(
                        msg.agent_id,
                        msg.module_type,
                        msg.position.x as i32, 
                        msg.position.y as i32,
                        msg.orientation as i32, 
                    ).await;
                });
            },
        )?;

        self.subscriptions.lock().unwrap().push(Arc::new(sub));
        info!("📡 ROS 2 Subscriptions aktiv.");
        Ok(())
    }
}