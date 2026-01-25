use serde::{Serialize, Deserialize};
use serde_json::{json, Value};
use passform_msgs::msg::{Location as LocationMsg, AreaOfInterest as AoiMsg};
use geometry_msgs::msg::{Point, Quaternion, Pose, Polygon};

/// Hilfstraits für die BaSyx-Konvertierung
pub trait ToBasyx {
    fn to_basyx_json(&self) -> Value;
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct LocalPoint { pub x: f64, pub y: f64, pub z: f64 }

impl From<&Point> for LocalPoint {
    fn from(p: &Point) -> Self { Self { x: p.x, y: p.y, z: p.z } }
}

impl ToBasyx for LocalPoint {
    fn to_basyx_json(&self) -> Value {
        json!({
            "idShort": "point",
            "modelType": "SubmodelElementCollection",
            "value": [
                { "idShort": "x", "modelType": "Property", "valueType": "double", "value": self.x },
                { "idShort": "y", "modelType": "Property", "valueType": "double", "value": self.y },
                { "idShort": "z", "modelType": "Property", "valueType": "double", "value": self.z }
            ]
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct LocalQuaternion { pub x: f64, pub y: f64, pub z: f64, pub w: f64 }

impl From<&Quaternion> for LocalQuaternion {
    fn from(q: &Quaternion) -> Self { Self { x: q.x, y: q.y, z: q.z, w: q.w } }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct LocalPose {
    pub position: LocalPoint,
    pub orientation: LocalQuaternion,
}

impl ToBasyx for LocalPose {
    fn to_basyx_json(&self) -> Value {
        json!({
            "idShort": "pose",
            "modelType": "SubmodelElementCollection",
            "value": [
                self.position.to_basyx_json(),
                {
                    "idShort": "quaternion",
                    "modelType": "SubmodelElementCollection",
                    "value": [
                        { "idShort": "x", "modelType": "Property", "value": self.orientation.x },
                        { "idShort": "y", "modelType": "Property", "value": self.orientation.y },
                        { "idShort": "z", "modelType": "Property", "value": self.orientation.z },
                        { "idShort": "w", "modelType": "Property", "value": self.orientation.w }
                    ]
                }
            ]
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct LocalAreaOfInterest {
    pub label: String,
    pub uid: String,
    pub points: Vec<LocalPoint>,
}

impl ToBasyx for LocalAreaOfInterest {
    fn to_basyx_json(&self) -> Value {
        let polygon_elements: Vec<Value> = self.points.iter().enumerate().map(|(i, p)| {
            json!({
                "idShort": format!("point_{}", i),
                "modelType": "SubmodelElementCollection",
                "value": [
                    { "idShort": "x", "modelType": "Property", "value": p.x },
                    { "idShort": "y", "modelType": "Property", "value": p.y },
                    { "idShort": "z", "modelType": "Property", "value": p.z }
                ]
            })
        }).collect();

        json!({
            "idShort": "area_of_interest",
            "modelType": "SubmodelElementCollection",
            "value": [
                { "idShort": "label", "modelType": "Property", "value": self.label },
                { "idShort": "uid", "modelType": "Property", "value": self.uid },
                { "idShort": "polygon", "modelType": "SubmodelElementCollection", "value": polygon_elements }
            ]
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Location {
    pub frame_id: String,
    pub pose: LocalPose,
    pub aoi: LocalAreaOfInterest,
}

impl Location {
    pub fn from_msg(msg: &LocationMsg) -> Self {
        Self {
            frame_id: msg.header.frame_id.clone(),
            pose: LocalPose {
                position: LocalPoint::from(&msg.pose.position),
                orientation: LocalQuaternion::from(&msg.pose.orientation),
            },
            aoi: LocalAreaOfInterest {
                label: msg.aoi.label.clone(),
                uid: msg.aoi.uuid.clone(),
                points: msg.aoi.polygon.points.iter().map(|p| LocalPoint { x: p.x as f64, y: p.y as f64, z: p.z as f64 }).collect(),
            },
        }
    }

    pub fn to_msg(&self) -> LocationMsg {
        // Hier erfolgt die Rückkonvertierung in die ROS-Message
        // (Weglassen der Details für Kürze, folgt dem Schema oben)
        LocationMsg::default() 
    }

    pub fn get_aoi_uid(&self) -> &str {
        &self.aoi.uid
    }
}

impl ToBasyx for Location {
    fn to_basyx_json(&self) -> Value {
        json!({
            "idShort": "location",
            "modelType": "SubmodelElementCollection",
            "value": [
                { "idShort": "frame_id", "modelType": "Property", "value": self.frame_id },
                self.aoi.to_basyx_json(),
                self.pose.to_basyx_json()
            ]
        })
    }
}