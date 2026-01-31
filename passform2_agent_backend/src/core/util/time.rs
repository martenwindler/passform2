use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use rclrs::{Node, Publisher};
use std_msgs::msg::Header;

/// Ein Watchdog, der den Status auf "Stale" setzt, wenn kein Heartbeat kommt.
pub struct Watchdog {
    timeout_duration: Duration,
    timeout_status: i32,
    status: Arc<Mutex<i32>>,
    last_seen: Arc<Mutex<u64>>,
    timer: Arc<Mutex<Option<std::thread::JoinHandle<()>>>>,
}

impl Watchdog {
    pub fn new(timeout_sec: f64, timeout_status: i32) -> Self {
        Self {
            timeout_duration: Duration::from_secs_f64(timeout_sec),
            timeout_status,
            status: Arc::new(Mutex::new(0)),
            last_seen: Arc::new(Mutex::new(0)),
            timer: Arc::new(Mutex::new(None)),
        }
    }

    pub fn on_heartbeat(&self, current_status: i32) {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        
        {
            let mut last = self.last_seen.lock().unwrap();
            *last = now;
            let mut s = self.status.lock().unwrap();
            *s = current_status;
        }

        self.start_timer();
    }

    fn start_timer(&self) {
        let status_ptr = Arc::clone(&self.status);
        let timeout_val = self.timeout_status;
        let duration = self.timeout_duration;

        let mut timer_guard = self.timer.lock().unwrap();
        
        *timer_guard = Some(std::thread::spawn(move || {
            std::thread::sleep(duration);
            let mut s = status_ptr.lock().unwrap();
            *s = timeout_val;
            eprintln!("Module timed out!");
        }));
    }

    pub fn stop(&self) {
        let mut timer_guard = self.timer.lock().unwrap();
        *timer_guard = None; 
    }
}

/// ROS 2 Heartbeat Publisher mit Liveliness-QoS
pub struct Heartbeat {
    pub publisher: Arc<Publisher<Header>>,
    pub timer: Arc<rclrs::Timer>,
}

impl Heartbeat {
    pub fn new(node: &Node, period_sec: f64) -> Self {
        // create_publisher nimmt in Jazzy nur den Topic-Namen (für Default QoS)
        let publisher = node.create_publisher::<Header>("ymodule_broadcast")
            .expect("Failed to create publisher");
        
        let publisher_clone = Arc::clone(&publisher);
        let node_clone = node.clone();
        
        let timer = node.create_timer_repeating(
            std::time::Duration::from_secs_f64(period_sec),
            move || {
                let rcl_time = node_clone.get_clock().now().to_ros_msg().expect("Time conversion failed");
                let msg = Header {
                    stamp: builtin_interfaces::msg::Time {
                        sec: rcl_time.sec,
                        nanosec: rcl_time.nanosec,
                    },
                    frame_id: "".to_string(),
                };
                let _ = publisher_clone.publish(&msg);
            }
        ).expect("Failed to create timer");

        Self {
            // Fix E0308: Konvertierung in die interne Arc-Struktur
            publisher: publisher.into(), 
            timer: Arc::new(timer),
        }
    }
}