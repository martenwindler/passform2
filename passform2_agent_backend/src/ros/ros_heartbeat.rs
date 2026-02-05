use std::sync::Arc;
use std::time::Duration;

// ROS 2 Core Imports
use rclrs::{Node, Publisher, Timer};

// Message Imports
// Hinweis: Sicherstellen, dass diese in deiner Cargo.toml unter [dependencies] stehen
use std_msgs::msg::Header;
use builtin_interfaces::msg::Time as RosTime;

/// ROS 2 Heartbeat Publisher mit Liveliness-QoS
/// Sendet regelmäßig einen Header-Zeitstempel auf dem "ymodule_broadcast" Topic.
pub struct Heartbeat {
    pub publisher: Arc<Publisher<Header>>,
    pub timer: Arc<Timer>,
}

impl Heartbeat {
    pub fn new(node: &Node, period_sec: f64) -> Self {
        // In Jazzy: Default QoS wird verwendet, wenn kein zweites Argument übergeben wird
        let publisher = node
            .create_publisher::<Header>("ymodule_broadcast")
            .expect("Failed to create heartbeat publisher");
        
        let publisher_clone = Arc::clone(&publisher);
        let node_clone = node.clone();
        
        let timer = node.create_timer_repeating(
            Duration::from_secs_f64(period_sec),
            move || {
                // Zeit vom Node-Clock holen und konvertieren
                if let Ok(rcl_time) = node_clone.get_clock().now().to_ros_msg() {
                    let msg = Header {
                        stamp: RosTime {
                            sec: rcl_time.sec,
                            nanosec: rcl_time.nanosec,
                        },
                        frame_id: "system_heartbeat".to_string(),
                    };
                    let _ = publisher_clone.publish(&msg);
                }
            },
        ).expect("Failed to create heartbeat timer");

        Self {
            // .into() konvertiert rclrs::Publisher in Arc<rclrs::Publisher>
            publisher: publisher.into(), 
            timer: Arc::new(timer),
        }
    }
}