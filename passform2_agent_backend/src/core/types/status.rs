use serde::{Serialize, Deserialize};
use num_derive::FromPrimitive;    
use num_traits::FromPrimitive;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, FromPrimitive)]
#[repr(i32)]
pub enum Status {
    Ok = 0,
    Warn = 1,
    Error = 2,
    Stale = 3,
    Running = 4,
}

impl From<i32> for Status {
    fn from(value: i32) -> Self {
        Status::from_i32(value).unwrap_or(Status::Error)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, FromPrimitive)]
#[repr(i32)]
pub enum ConnectionStatus {
    Connected = 0,
    Disconnected = 1,
    Timeout = 2,
    Unknown = 3,
}