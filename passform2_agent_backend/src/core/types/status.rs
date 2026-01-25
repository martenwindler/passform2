use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[repr(i32)]
pub enum Status {
    Ok = 0,
    Warn = 1,
    Error = 2,
    Stale = 3,
    Running = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i32)]
pub enum ConnectionStatus {
    Connected = 0,
    Disconnected = 1,
    Timeout = 2,
    UnknownError = 3,
}

// Hilfsimplementierung für die Konvertierung von Integern (z.B. aus ROS-Messages)
impl From<i32> for Status {
    fn from(value: i32) -> Self {
        match value {
            0 => Status::Ok,
            1 => Status::Warn,
            2 => Status::Error,
            3 => Status::Stale,
            4 => Status::Running,
            _ => Status::Error, // Fallback
        }
    }
}