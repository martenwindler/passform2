#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from passform_agent_msgs.msg import (
    AgentAnnounce,
    PathRequest,
    PathComplete,
    AgentInfo,
)
from std_msgs.msg import Float32  # Für den SSoT-Systemtakt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        # --- ZUSTAND ---
        self.known_agents = {}      # aid -> AgentInfo
        self.last_seen = {}         # aid -> float (Timestamp)
        self.agent_signals = {}     # aid -> int (0-100)
        self.current_hz = 1.0       # Standard-Takt (SSoT)
        
        self.current_request = None
        self.request_info = {}
        self.prev_marker_ids = []

        # --- QOS & SUBSCRIPTIONS ---
        announce_qos = QoSProfile(
            depth=10, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL, 
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.create_subscription(
            AgentAnnounce, 'agent_announce', self.on_announce, announce_qos)
        
        self.create_subscription(
            Float32, 'system/heartbeat_rate', self.on_hz_update, 10)
            
        self.create_subscription(
            PathRequest, 'path_request', self.on_request, 10)
        self.create_subscription(
            PathComplete, 'path_complete', self.on_complete, 10)

        self.vis_pub = self.create_publisher(
            MarkerArray, 'visualization_marker_array', announce_qos)

        # Timer für Health-Check (2Hz Abfrage)
        self.create_timer(0.5, self.check_agents_health)

    def on_hz_update(self, msg):
        self.current_hz = msg.data
        self.get_logger().info(f"Monitor: SSoT-Takt auf {self.current_hz} Hz aktualisiert.")

    def on_announce(self, msg: AgentAnnounce):
        aid = msg.agent_info.agent_id
        if msg.active:
            self.known_agents[aid] = msg.agent_info
            self.last_seen[aid] = time.time()
            self.agent_signals[aid] = 100
        else:
            self.cleanup_agent(aid)
        self.publish_markers()

    def cleanup_agent(self, aid):
        if aid in self.known_agents: del self.known_agents[aid]
        if aid in self.last_seen: del self.last_seen[aid]
        if aid in self.agent_signals: del self.agent_signals[aid]

    def check_agents_health(self):
        now = time.time()
        # Timeout nach 3 verpassten Intervallen basierend auf Hz
        threshold = (1.0 / self.current_hz) * 3.0
        
        changed = False
        for aid in list(self.known_agents.keys()):
            last_ts = self.last_seen.get(aid, 0)
            diff = now - last_ts
            
            strength = max(0, int(100 * (1 - (diff / threshold))))
            
            if strength != self.agent_signals.get(aid):
                self.agent_signals[aid] = strength
                changed = True

        if changed:
            self.publish_markers()

    def on_request(self, msg: PathRequest):
        rid = msg.request_id
        self.current_request = rid
        self.request_info[rid] = {
            "start": (msg.start.x, msg.start.y),
            "goal": (msg.goal.x, msg.goal.y),
            "path": [],
            "complete": False
        }
        self.publish_markers()

    def on_complete(self, msg: PathComplete):
        rid = msg.request_id
        if rid == self.current_request and rid in self.request_info:
            self.request_info[rid]["path"] = [(gp.position.x, gp.position.y) for gp in msg.path]
            self.request_info[rid]["complete"] = True
            self.publish_markers()

    def publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Löschen alter Marker (Cleanup für RViz)
        deletions = MarkerArray()
        for ns, mid in self.prev_marker_ids:
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', now
            m.ns, m.id, m.action = ns, mid, Marker.DELETE
            deletions.markers.append(m)
        if deletions.markers: self.vis_pub.publish(deletions)

        new_ids = []
        mid = 0

        # FARBMAP FÜR ALLE AGENTEN-TYPEN
        color_map = {
            'greifer':   (1.0, 0.0, 0.0), # Rot
            'mensch':    (0.0, 0.0, 1.0), # Blau
            'rollen_ns': (0.0, 1.0, 0.0), # Grün
            'rollen_ow': (1.0, 1.0, 0.0), # Gelb
            'conveyeur': (1.0, 0.5, 0.0), # Orange
            'tisch':     (0.6, 0.6, 0.6), # Grau
            'ftf':       (0.0, 1.0, 1.0), # Cyan (Transportroboter)
        }

        # RICHTUNGSMAP (Pfeile)
        # Mensch und Greifer erhalten Pfeile in alle 4 Richtungen
        dir_map = {
            'mensch':    [(0, 0.4), (0, -0.4), (0.4, 0), (-0.4, 0)],
            'greifer':   [(0, 0.4), (0, -0.4), (0.4, 0), (-0.4, 0)],
            'ftf':       [(0, 0.4), (0, -0.4), (0.4, 0), (-0.4, 0)],
            'conveyeur': [(0.4, 0), (-0.4, 0)], 
            'rollen_ns': [(0, 0.4), (0, -0.4)],
            'rollen_ow': [(0.4, 0), (-0.4, 0)],
        }

        for aid, info in self.known_agents.items():
            strength = self.agent_signals.get(aid, 0)
            x, y = info.position.x, info.position.y
            mt = info.module_type
            
            # 1. CUBE (Physischer Körper des Agenten)
            cube = Marker()
            cube.header.frame_id, cube.header.stamp = 'map', now
            cube.ns, cube.id, cube.type = 'agents', mid, Marker.CUBE
            cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = float(x), float(y), 0.1
            cube.scale.x = cube.scale.y = 0.8
            cube.scale.z = 0.2
            
            r, g, b = color_map.get(mt, (1.0, 1.0, 1.0))
            cube.color.r, cube.color.g, cube.color.b = r, g, b
            cube.color.a = 1.0 if strength > 20 else 0.3 # Transparent bei Signalverlust
            
            ma.markers.append(cube)
            new_ids.append(('agents', mid))
            mid += 1

            # 2. RICHTUNGS-PFEILE (Flow-Anzeige)
            for dx, dy in dir_map.get(mt, []):
                arr = Marker()
                arr.header.frame_id, arr.header.stamp = 'map', now
                arr.ns, arr.id, arr.type = 'directions', mid, Marker.ARROW
                arr.points = [Point(x=float(x), y=float(y), z=0.25), Point(x=float(x+dx), y=float(y+dy), z=0.25)]
                arr.scale.x, arr.scale.y, arr.scale.z = 0.05, 0.1, 0.1
                arr.color.r = arr.color.g = arr.color.b = 0.9
                arr.color.a = 0.6 if strength > 20 else 0.1
                ma.markers.append(arr)
                new_ids.append(('directions', mid))
                mid += 1

            # 3. TEXT-LABEL (Typ + Signalstärke in %)
            label = Marker()
            label.header.frame_id, label.header.stamp = 'map', now
            label.ns, label.id, label.type = 'labels', mid, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = float(x), float(y), 0.65
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0 if strength > 0 else 0.3
            label.text = f"{mt.upper()}\n{strength}%"
            ma.markers.append(label)
            new_ids.append(('labels', mid))
            mid += 1

        if ma.markers:
            self.vis_pub.publish(ma)
        self.prev_marker_ids = new_ids

def main():
    rclpy.init()
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()