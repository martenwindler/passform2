#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)] // Erlaubt die Darstellung als Integer wie in Python
pub enum LogLevel {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
    Fatal = 4,
}

impl From<i32> for LogLevel {
    fn from(value: i32) -> Self {
        match value {
            0 => LogLevel::Debug,
            1 => LogLevel::Info,
            2 => LogLevel::Warn,
            3 => LogLevel::Error,
            _ => LogLevel::Fatal,
        }
    }
}