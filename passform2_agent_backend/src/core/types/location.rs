use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
pub struct LocalPoint { 
    pub x: f64, 
    pub y: f64, 
    pub z: f64 
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
pub struct LocalQuaternion { 
    pub x: f64, 
    pub y: f64, 
    pub z: f64, 
    pub w: f64 
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
pub struct LocalPose {
    pub position: LocalPoint,
    pub orientation: LocalQuaternion,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
pub struct LocalAreaOfInterest {
    pub label: String,
    pub uid: String,
    pub points: Vec<LocalPoint>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
pub struct WorldLocation {
    pub frame_id: String,
    pub pose: LocalPose,
    pub aoi: LocalAreaOfInterest,
}

impl WorldLocation {
    /// Hilfsmethode für den schnellen Zugriff auf die AOI-ID
    pub fn get_aoi_uid(&self) -> &str {
        &self.aoi.uid
    }
}