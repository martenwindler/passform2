#!/usr/bin/env python3
import eventlet
eventlet.monkey_patch()  # WICHTIG: Muss die allererste Zeile sein!

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import socketio
import threading
from passform_agent_msgs.msg import AgentAnnounce

# --- SOCKET.IO SETUP ---
sio = socketio.Server(
    cors_allowed_origins='*', 
    async_mode='eventlet',
    ping_timeout=60,
    ping_interval=25,
    always_connect=True 
)
app = socketio.WSGIApp(sio)

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')
        self.agents_cache = {}

        # --- FLEXIBLES QoS PROFIL ---
        # Wir nutzen BEST_EFFORT und VOLATILE, um sicherzustellen, dass wir 
        # die Nachrichten des Test-Nodes auf jeden Fall empfangen.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            AgentAnnounce, 
            'agent_announce', 
            self.on_agent_announce, 
            qos_profile
        )

        self.get_logger().info("🚀 ROS-Bridge aktiv. Höre auf 'agent_announce' (Tolerantes QoS)")

    def on_agent_announce(self, msg):
        # Wenn diese Funktion aufgerufen wird, sehen wir es sofort im Terminal
        info = msg.agent_info
        aid = info.agent_id
        
        print(f"📥 ROS-DATEN EMPFANGEN: {aid} an ({info.position.x}, {info.position.y})")

        if msg.active:
            key = f"{info.position.x},{info.position.y}"
            self.agents_cache[key] = {
                "agent_id": aid,
                "module_type": info.module_type,
                "position": {"x": info.position.x, "y": info.position.y},
                "orientation": 0,
                "is_dynamic": info.module_type in ["ftf", "mensch"],
                "payload": None,
                "signal_strength": 100
            }
        else:
            self.agents_cache = {k: v for k, v in self.agents_cache.items() if v["agent_id"] != aid}

        # Daten an Frontend senden
        sio.emit('active_agents', self.agents_cache)

# --- SOCKET.IO EVENTS ---

@sio.event
def connect(sid, environ):
    print(f"🔗 Browser verbunden (SID: {sid})")

@sio.on('set_heartbeat_rate')
def on_set_hz(sid, data):
    print(f"⚙️ Takt-Kommando: {data} Hz")

# --- EXECUTION LOGIK ---

def start_ros():
    if not rclpy.ok():
        rclpy.init()
    node = WebBridgeNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"🔥 ROS-Fehler: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # 1. ROS-Client im Hintergrund
    ros_thread = threading.Thread(target=start_ros, daemon=True)
    ros_thread.start()

    # 2. Webserver
    print("🌐 Bridge-Server auf Port 5000 bereit.")
    try:
        eventlet.wsgi.server(
            eventlet.listen(('', 5000)), 
            app, 
            log_output=True
        )
    except KeyboardInterrupt:
        print("\n🛑 Bridge beendet.")