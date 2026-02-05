// src/core/types/mod.rs

// 1. Untermodule deklarieren
pub mod plant_model;
pub mod bay_aas;
pub mod inventory;
pub mod location;
pub mod module;
pub mod status;
pub mod agent_entry;

pub use self::status::{Status, ConnectionStatus};
pub use self::bay_aas::Bay;
pub use self::agent_entry::AgentEntry;

use serde::{Serialize, Deserialize};
use num_derive::FromPrimitive;

// --- SKILL TYPEN ---
// SkillType bleibt hier, da es ein übergreifender Typ ist.

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, FromPrimitive)]
#[repr(i32)]
pub enum SkillType {
    // Basis & Bewegung (0-9)
    Stop = 0, 
    Move = 1, 
    MoveTcp = 2, 
    MoveBody = 3,
    // Greifer & Aktoren (10-19)
    Grasp = 10, 
    Pressure = 12, 
    Position = 13, 
    Release = 14,
    // Werkzeug-Management (20-29)
    ToolGet = 20, 
    ToolUse = 21, 
    ToolPut = 22, 
    Store = 23, 
    Provide = 24,
    // Montage-Operationen (30-39)
    AssemblyPickPlace = 30, 
    AssemblyScrew = 31, 
    AssemblyMount = 32, 
    AssemblyPlug = 33,
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