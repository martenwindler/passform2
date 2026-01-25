use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::thread;
use std::time::Duration;

pub struct Watchdog {
    timeout: Duration,
    callback: Arc<dyn Fn() + Send + Sync>,
    is_running: Arc<AtomicBool>,
    reset_flag: Arc<AtomicBool>,
    handle: Mutex<Option<thread::JoinHandle<()>>>,
}

impl Watchdog {
    pub fn new<F>(timeout_secs: f32, callback: F) -> Self 
    where 
        F: Fn() + Send + Sync + 'static 
    {
        let watchdog = Self {
            timeout: Duration::from_secs_f32(timeout_secs),
            callback: Arc::new(callback),
            is_running: Arc::new(AtomicBool::new(true)),
            reset_flag: Arc::new(AtomicBool::new(false)),
            handle: Mutex::new(None),
        };

        watchdog.start_internal();
        watchdog
    }

    fn start_internal(&self) {
        let timeout = self.timeout;
        let callback = Arc::clone(&self.callback);
        let is_running = Arc::clone(&self.is_running);
        let reset_flag = Arc::clone(&self.reset_flag);

        let mut handle_guard = self.handle.lock().unwrap();
        *handle_guard = Some(thread::spawn(move || {
            while is_running.load(Ordering::SeqCst) {
                // Der eigentliche Watchdog-Check
                // Wir schlafen in kleinen Intervallen, um schneller auf Resets/Stops zu reagieren
                let start_time = std::time::Instant::now();
                while start_time.elapsed() < timeout {
                    if reset_flag.swap(false, Ordering::SeqCst) {
                        // Reset wurde getriggert, fange Zeitmessung von vorne an
                        break;
                    }
                    if !is_running.load(Ordering::SeqCst) {
                        return;
                    }
                    thread::sleep(Duration::from_millis(50));
                }

                // Wenn wir hier ankommen und die Zeit abgelaufen ist ohne Reset...
                if start_time.elapsed() >= timeout {
                    (callback)();
                    // Nachdem der Callback lief, beenden wir den Watchdog meistens
                    is_running.store(false, Ordering::SeqCst);
                }
            }
        }));
    }

    /// Setzt den Timer zurück (Heartbeat erhalten)
    pub fn reset(&self) {
        self.reset_flag.store(true, Ordering::SeqCst);
    }

    /// Beendet den Watchdog sauber
    pub fn stop(&self) {
        self.is_running.store(false, Ordering::SeqCst);
        let mut handle_guard = self.handle.lock().unwrap();
        if let Some(h) = handle_guard.take() {
            let _ = h.join();
        }
    }
}

// Implementierung für das Drop-Trait (Sicherheit beim Löschen des Objekts)
impl Drop for Watchdog {
    fn drop(&mut self) {
        self.stop();
    }
}