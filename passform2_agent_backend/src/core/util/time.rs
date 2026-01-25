use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use rclrs::{Node, Publisher, QosProfile, LivelinessPolicy};
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
        // Zeitstempel aktualisieren
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
        
        // In Rust stoppen wir keinen Thread hart, wir lassen ihn auslaufen oder nutzen 
        // koordinierte Timer. Hier simulieren wir den Python-Restart-Timer:
        *timer_guard = Some(std::thread::spawn(move || {
            std::thread::sleep(duration);
            let mut s = status_ptr.lock().unwrap();
            *s = timeout_val;
            // Hier würde man in Rust eher ein Event triggern statt eine Exception zu werfen,
            // da Exceptions in Threads den Hauptprozess nicht sauber beenden.
            eprintln!("Module timed out!");
        }));
    }

    pub fn stop(&self) {
        let mut timer_guard = self.timer.lock().unwrap();
        *timer_guard = None; // Handle droppen
    }
}

/// ROS 2 Heartbeat Publisher mit Liveliness-QoS
pub struct Heartbeat {
    publisher: Arc<Publisher<Header>>,
    _timer: Arc<rclrs::Timer>,
}

impl Heartbeat {
    pub fn new(node: &Arc<Node>, period_sec: f64) -> Self {
        let lease_delta = 0.02; // 20 ms
        let total_duration = period_sec + lease_delta;

        // QoS-Profil wie im Python-Original
        let mut qos = QosProfile::default();
        qos.liveliness = LivelinessPolicy::ManualByTopic;
        qos.liveliness_lease_duration = Duration::from_secs_f64(total_duration);
        qos.deadline = Duration::from_secs_f64(total_duration);

        let publisher = node.create_publisher::<Header>("ymodule_broadcast", qos).unwrap();
        
        let publisher_clone = Arc::clone(&publisher);
        let node_clone = Arc::clone(node);
        
        let timer = node.create_timer(Duration::from_secs_f64(period_sec), move || {
            let msg = Header {
                stamp: node_clone.get_clock().now().to_msg(),
                frame_id: "".to_string(),
            };
            publisher_clone.publish(&msg).unwrap();
        }).unwrap();

        Self {
            publisher,
            _timer: timer,
        }
    }
}