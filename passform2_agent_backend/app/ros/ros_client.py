#!/usr/bin/env python3
import os
import threading
import logging
from typing import Dict, Optional, Any, List

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.timer import Timer

from passform_agent_resources.msg import AgentAnnounce, AgentInfo, PathRequest, PathComplete
from app.config import config
from app.managers.agent_manager import agent_manager

logger = logging.getLogger("ros_client")

class RosClient(Node):
    def __init__(self) -> None:
        # Domain ID aus Config setzen (0 für HW, 1 für Sim)
        os.environ["ROS_DOMAIN_ID"] = str(config.get_domain_id())

        super().__init__("python_backend_node")
        self.get_logger().info(
            f"📡 ROS Client Online | Modus: {config.get_current_mode().value} | Domain: {config.get_domain_id()}"
        )

        self.heartbeat_timeout_sec = 5.0  # Etwas toleranterer Timeout
        self.heartbeat_timers: Dict[str, Timer] = {}

        self._setup_ros_interfaces()

    def _setup_ros_interfaces(self) -> None:
        # Subscriber für Live-Daten
        self.agent_info_sub = self.create_subscription(
            AgentInfo, "agent_info", self._on_agent_info, 10
        )
        self.agent_announce_sub = self.create_subscription(
            AgentAnnounce, "agent_announce", self._on_agent_announce, 10
        )
        
        # Pfad-Interfaces
        self.path_request_pub = self.create_publisher(PathRequest, "path_request", 10)
        self.path_complete_sub = self.create_subscription(
            PathComplete, "path_complete", self._on_path_complete, 10
        )

    def _on_agent_info(self, msg: AgentInfo) -> None:
        """Live-Positions-Update von einem ROS-Node."""
        agent_manager.sync_from_ros(msg)
        self._manage_heartbeat(msg.agent_id)

    def _on_agent_announce(self, msg: AgentAnnounce) -> None:
        """An- oder Abmeldung eines Agenten."""
        agent_manager.sync_from_ros(msg.agent_info)
        
        if not msg.active:
            logger.info(f"👋 Agent meldet sich ab: {msg.agent_info.agent_id}")
            agent_manager.remove_agent(msg.agent_info.agent_id)
        else:
            self._manage_heartbeat(msg.agent_info.agent_id)

    def _manage_heartbeat(self, agent_id: str) -> None:
        """Überwacht, ob Agenten noch senden (besonders wichtig für FTFs)."""
        if agent_id in self.heartbeat_timers:
            self.heartbeat_timers[agent_id].cancel()
            self.destroy_timer(self.heartbeat_timers[agent_id])

        def _on_timeout():
            # Nur entfernen, wenn es kein statisches SSoT-Modul ist oder wir im Sim-Modus sind
            self.get_logger().warning(f"⚠️ Heartbeat-Loss: {agent_id}")
            agent_manager.remove_agent(agent_id)

        self.heartbeat_timers[agent_id] = self.create_timer(
            self.heartbeat_timeout_sec, _on_timeout
        )

    def _on_path_complete(self, msg: PathComplete) -> None:
        """Wird aufgerufen, wenn ein Hardware-Node eine Teil-Trajektorie fertig berechnet hat."""
        # Hier könnte man die Daten an den PathManager in Elm zurückgeben
        pass

# --- Infrastruktur zum Starten ---

_instance: Optional[RosClient] = None
_executor: Optional[SingleThreadedExecutor] = None
_thread: Optional[threading.Thread] = None

def get_ros_client() -> RosClient:
    global _instance, _executor, _thread
    if _instance is None:
        if not rclpy.ok():
            rclpy.init()
        _instance = RosClient()
        _executor = SingleThreadedExecutor()
        _executor.add_node(_instance)
        _thread = threading.Thread(target=_executor.spin, daemon=True)
        _thread.start()
    return _instance

def restart_ros_client():
    """Wird bei Modus-Wechsel aufgerufen."""
    global _instance, _executor
    if _instance:
        _instance.get_logger().info("🔄 Restarting ROS Client...")
        # Hier Node-Shutdown-Logik
        _instance = None
    return get_ros_client()