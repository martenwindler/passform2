// Automatisch generiert durch fix__rust_modules.sh
pub mod bay;
pub mod bay_aas;
pub mod inventory;
pub mod location;
pub mod master;
pub mod module;
pub mod parameter;
pub mod status;

use serde::{Serialize, Deserialize};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

// --- STATUS TYPEN ---

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

// --- SKILL TYPEN ---

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, FromPrimitive)]
#[repr(i32)]
pub enum SkillType {
    // Basis & Bewegung (0-9)
    Stop = 0, Move = 1, MoveTcp = 2, MoveBody = 3,
    // Greifer & Aktoren (10-19)
    Grasp = 10, Pressure = 12, Position = 13, Release = 14,
    // Werkzeug-Management (20-29)
    ToolGet = 20, ToolUse = 21, ToolPut = 22, Store = 23, Provide = 24,
    // Montage-Operationen (30-39)
    AssemblyPickPlace = 30, AssemblyScrew = 31, AssemblyMount = 32, AssemblyPlug = 33,
}

impl SkillType {
    /// Gibt den standardisierten Namen für das BaSyx idShort zurück
    pub fn as_str(&self) -> &'static str {
        match self {
            SkillType::Stop => "StopOperation",
            SkillType::Move | SkillType::MoveTcp | SkillType::MoveBody => "MoveOperation",
            SkillType::Grasp => "GraspOperation",
            SkillType::Release => "ReleaseOperation",
            SkillType::ToolGet => "ToolGetOperation",
            SkillType::AssemblyPickPlace => "PickPlaceOperation",
            _ => "GenericOperation",
        }
    }
}