#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from passform_agent_msgs.msg import (
    AgentAnnounce,
    PathRequest,
    PathComplete,
    AgentInfo,
)
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        # Aktueller Zustand
        # agent_id -> AgentInfo
        self.known_agents    : dict[str, AgentInfo] = {}
        self.current_request : str | None    = None
        self.request_info    : dict[str,dict] = {}

        # Für Marker-Löschung
        self.prev_marker_ids = []  # list of (ns,id) tuples

        announce_qos = QoSProfile(depth=10)
        announce_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        announce_qos.reliability = ReliabilityPolicy.RELIABLE
        # Subscriptions & Publisher
        self.create_subscription(
            AgentAnnounce, 'agent_announce', self.on_announce, announce_qos)
        self.create_subscription(
            PathRequest,   'path_request',   self.on_request, 10)
        self.create_subscription(
            PathComplete,  'path_complete',  self.on_complete, 10)

        self.vis_pub = self.create_publisher(
            MarkerArray, 'visualization_marker_array', announce_qos)

    def on_announce(self, msg: AgentAnnounce):
        aid = msg.agent_info.agent_id
        if msg.active:
            # Neue Anmeldung?
            if aid not in self.known_agents:
                info = msg.agent_info

                x = info.position.x; y = info.position.y; mt = info.module_type
                self.get_logger().info(
                    f"Agent online: {aid} at ({x},{y}), type={mt}"
                )
                # Speichere die gesamte AgentInfo
                self.known_agents[aid] = info
                self.publish_markers()
        else:
            # Abmeldung?
            if aid in self.known_agents:
                self.get_logger().info(f"Agent offline: {aid}")
                del self.known_agents[aid]
                self.publish_markers()

    def on_request(self, msg: PathRequest):
        rid = msg.request_id
        self.current_request = rid
        self.request_info = {
            rid: {
                "start":    (msg.start.x, msg.start.y),
                "goal":     (msg.goal.x,  msg.goal.y),
                "path":     [],
                "complete": False
            }
        }
        self.get_logger().info(
            f"Received PathRequest {rid}: "
            f"start={self.request_info[rid]['start']} "
            f"→ goal={self.request_info[rid]['goal']}"
        )
        self.publish_markers()

    def on_complete(self, msg: PathComplete):
        rid = msg.request_id
        if rid == self.current_request and rid in self.request_info:
            pts = [(gp.position.x, gp.position.y) for gp in msg.path]
            self.request_info[rid]["path"]     = pts
            self.request_info[rid]["complete"] = True
            self.get_logger().info(
                f"Received PathComplete {rid}: {len(pts)} steps"
            )
            self.publish_markers()

    def publish_markers(self):
        ma = MarkerArray()
        deletions = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1) Alte Marker löschen
        for ns, mid in self.prev_marker_ids:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = now
            m.ns     = ns
            m.id     = mid
            m.action = Marker.DELETE
            deletions.markers.append(m)
        if deletions.markers:
            self.vis_pub.publish(deletions)

        # 2) Neuzeichnen
        new_ids = []
        mid = 0

        # Farben je Modultyp
        color_map = {
            'greifer':   (1.0, 0.0, 0.0),
            'mensch':    (0.0, 0.0, 1.0),
            'rollen_ns': (0.0, 1.0, 0.0),
            'rollen_ow': (1.0, 1.0, 0.0),
        }

        # --- Agents als Würfel ---
        for aid, info in self.known_agents.items():
            x = info.position.x; y = info.position.y; mt = info.module_type
            cube = Marker()
            cube.header.frame_id = 'map'
            cube.header.stamp    = now
            cube.ns     = 'agents'
            cube.id     = mid
            cube.type   = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = float(x)
            cube.pose.position.y = float(y)
            cube.pose.position.z = 0.1
            cube.scale.x = cube.scale.y = 0.8
            cube.scale.z = 0.2
            r, g, b = color_map.get(mt, (1.0,1.0,1.0))
            cube.color.r = r; cube.color.g = g; cube.color.b = b; cube.color.a = 1.0
            ma.markers.append(cube)
            new_ids.append(('agents', mid))
            mid += 1

        # --- Labels über den Agents (Typ + Koordinaten) ---
        for aid, info in self.known_agents.items():
            x = info.position.x; y = info.position.y; mt = info.module_type
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp    = now
            label.ns     = 'labels'
            label.id     = mid
            label.type   = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x)
            label.pose.position.y = float(y)
            label.pose.position.z = 0.5
            label.scale.z = 0.3
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"{mt}\n({x},{y})"
            ma.markers.append(label)
            new_ids.append(('labels', mid))
            mid += 1

        # --- Richtungs-Pfeile für Agents ---
        dir_map = {
            'greifer':   [(0,0.4),(0,-0.4),(0.4,0),(-0.4,0)],
            'mensch':    [(0,0.4),(0,-0.4),(0.4,0),(-0.4,0)],
            'rollen_ns': [(0,0.4),(0,-0.4)],
            'rollen_ow': [(0.4,0),(-0.4,0)],
        }
        for aid, info in self.known_agents.items():
            x = info.position.x; y = info.position.y; mt = info.module_type
            for dx, dy in dir_map.get(mt, []):
                arr = Marker()
                arr.header.frame_id = 'map'
                arr.header.stamp    = now
                arr.ns     = 'directions'
                arr.id     = mid
                arr.type   = Marker.ARROW
                arr.action = Marker.ADD
                start = Point(x=float(x),    y=float(y),    z=0.25)
                end   = Point(x=float(x+dx), y=float(y+dy), z=0.25)
                arr.points = [start, end]
                arr.scale.x = 0.05
                arr.scale.y = 0.1
                arr.scale.z = 0.1
                arr.color.r = arr.color.g = arr.color.b = 0.8
                arr.color.a = 0.6
                ma.markers.append(arr)
                new_ids.append(('directions', mid))
                mid += 1

        # --- Aktuellen Pfad als Pfeile ---
        if self.current_request:
            info = self.request_info[self.current_request]
            path = info["path"]
            for (x0, y0), (x1, y1) in zip(path, path[1:]):
                pa = Marker()
                pa.header.frame_id = 'map'
                pa.header.stamp    = now
                pa.ns     = 'path'
                pa.id     = mid
                pa.type   = Marker.ARROW
                pa.action = Marker.ADD
                pa.points = [
                    Point(x=float(x0), y=float(y0), z=0.4),
                    Point(x=float(x1), y=float(y1), z=0.4)
                ]
                pa.scale.x = 0.1
                pa.scale.y = 0.2
                pa.scale.z = 0.2
                if info["complete"]:
                    pa.color.r = pa.color.g = pa.color.b = 0.5
                    pa.color.a = 1.0
                else:
                    pa.color.r = pa.color.b = 0.0
                    pa.color.g = 1.0
                    pa.color.a = 1.0
                ma.markers.append(pa)
                new_ids.append(('path', mid))
                mid += 1

        # 3) Alles auf einmal veröffentlichen
        if ma.markers:
            self.vis_pub.publish(ma)

        # 4) IDs merken für spätere Löschungen
        self.prev_marker_ids = new_ids

def main():
    rclpy.init()
    node = MonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
