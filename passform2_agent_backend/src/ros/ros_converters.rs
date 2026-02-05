use crate::core::types::location::{WorldLocation, LocalPoint, LocalPose, LocalQuaternion, LocalAreaOfInterest};
use passform_agent_resources::msg::{WorldLocation as LocationMsg, WorldAreaOfInterest as AoiMsg};
use geometry_msgs::msg::{Point, Quaternion, Pose, Polygon, Point32};
use std_msgs::msg::Header;

pub struct RosConverter;

impl RosConverter {
    pub fn location_from_msg(msg: &LocationMsg) -> WorldLocation {
        WorldLocation {
            frame_id: msg.header.frame_id.clone(),
            pose: LocalPose {
                position: LocalPoint { x: msg.pose.position.x, y: msg.pose.position.y, z: msg.pose.position.z },
                orientation: LocalQuaternion { 
                    x: msg.pose.orientation.x, 
                    y: msg.pose.orientation.y, 
                    z: msg.pose.orientation.z, 
                    w: msg.pose.orientation.w 
                },
            },
            aoi: LocalAreaOfInterest {
                label: msg.aoi.label.clone(),
                uid: msg.aoi.uuid.clone(),
                points: msg.aoi.polygon.points.iter().map(|p| LocalPoint { 
                    x: p.x as f64, 
                    y: p.y as f64, 
                    z: p.z as f64 
                }).collect(),
            },
        }
    }

    pub fn location_to_msg(loc: &WorldLocation) -> LocationMsg {
        LocationMsg {
            header: Header {
                frame_id: loc.frame_id.clone(),
                ..Default::default()
            },
            pose: Pose {
                position: Point { x: loc.pose.position.x, y: loc.pose.position.y, z: loc.pose.position.z },
                orientation: Quaternion { 
                    x: loc.pose.orientation.x, 
                    y: loc.pose.orientation.y, 
                    z: loc.pose.orientation.z, 
                    w: loc.pose.orientation.w 
                },
            },
            aoi: AoiMsg {
                label: loc.aoi.label.clone(),
                uuid: loc.aoi.uid.clone(),
                polygon: Polygon {
                    points: loc.aoi.points.iter().map(|p| Point32 { 
                        x: p.x as f32, 
                        y: p.y as f32, 
                        z: p.z as f32 
                    }).collect(),
                },
            },
        }
    }
}