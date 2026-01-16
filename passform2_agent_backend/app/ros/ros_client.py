#!/usr/bin/env python3
"""
Zentraler ROS 2 Client des Backends.
Verwaltet die Kommunikation zwischen dem Web-Backend und dem ROS-Netzwerk.
"""

from __future__ import annotations
import os
import threading
from typing import Dict, Optional, Any, List

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.timer import Timer

from rosidl_runtime_py import message_to_ordereddict

# ROS Nachrichten-Typen laut passform_agent_msgs
from passform_agent_msgs.msg import (
    AgentAnnounce,
    AgentInfo,
    PathRequest,
    PathComplete,
)

from app.config import Config
from app.managers.agent_manager import agent_manager

# ====================================================================== #
#                                 NODE                                   #
# ====================================================================== #
class RosClient(Node):
    def __init__(self) -> None:
        # Domain ID setzen
        os.environ["ROS_DOMAIN_ID"] = str(Config.get_domain_id())

        super().__init__("python_backend_node")
        self.get_logger().info(
            f"ROS Client gestartet – Modus: {Config.get_current_mode().value}, "
            f"Domain: {Config.get_domain_id()}"
        )

        # Heartbeat Management
        self.heartbeat_timeout_sec = 2.0  
        self.heartbeat_timers: Dict[str, Timer] = {}

        self._setup_ros_interfaces()

    def _setup_ros_interfaces(self) -> None:
        """Abonniert Themen und erstellt Publisher."""
        
        # 1. Subscriber für direkte Status-Updates (AgentInfo)
        self.agent_info_sub = self.create_subscription(
            AgentInfo, "agent_info", self._on_agent_info, 10
        )

        # 2. Subscriber für Agenten-Ankündigungen (AgentAnnounce)
        self.agent_announce_sub = self.create_subscription(
            AgentAnnounce, "agent_announce", self._on_agent_announce, 10
        )

        # 3. Publisher für Pfadanfragen
        self.path_request_pub = self.create_publisher(PathRequest, "path_request", 10)
        self.path_request_central_pub = self.create_publisher(PathRequest, "path_request_central", 10)

        # 4. Subscriber für fertige Pfade
        self.path_complete_sub = self.create_subscription(
            PathComplete, "path_complete", self._on_path_complete, 10
        )

        self.get_logger().info("ROS-Interfaces erfolgreich eingerichtet")

    # ------------------------------------------------------------------ #
    #                          Agenten-Logik                             #
    # ------------------------------------------------------------------ #
    
    def _on_agent_info(self, msg: AgentInfo) -> None:
        """Verarbeitet direkte Heartbeats vom Typ AgentInfo."""
        # Wir reichen die Message direkt an den Manager weiter
        # Dieser extrahiert jetzt intern x, y und agent_id
        agent_manager.sync_from_ros(msg)
        self._manage_heartbeat(msg.agent_id)

    def _on_agent_announce(self, msg: AgentAnnounce) -> None:
        """Verarbeitet Heartbeats vom Typ AgentAnnounce."""
        # AgentAnnounce enthält ein AgentInfo-Feld namens 'agent_info'
        agent_manager.sync_from_ros(msg.agent_info)
        
        # Falls der Agent sich explizit abmeldet
        if not msg.active:
            agent_manager.remove_agent(msg.agent_info.agent_id)
        else:
            self._manage_heartbeat(msg.agent_info.agent_id)

    def _manage_heartbeat(self, agent_id: str) -> None:
        """Startet oder resettet den Timeout-Timer für einen Agenten."""
        if agent_id in self.heartbeat_timers:
            self.heartbeat_timers[agent_id].cancel()
            self.destroy_timer(self.heartbeat_timers[agent_id])

        def _on_timeout():
            self.get_logger().warning(f"⚠️ Agent offline (Timeout): {agent_id}")
            agent_manager.remove_agent(agent_id)
            if agent_id in self.heartbeat_timers:
                del self.heartbeat_timers[agent_id]

        self.heartbeat_timers[agent_id] = self.create_timer(
            self.heartbeat_timeout_sec, _on_timeout
        )

    # ------------------------------------------------------------------ #
    #                        Pfadplanungs-Logik                          #
    # ------------------------------------------------------------------ #
    
    def send_path_request(self, req: Any, use_central: bool = True) -> None:
        """Konvertiert Pydantic-Request in ROS-Message und sendet sie."""
        try:
            ros_msg = PathRequest()
            ros_msg.request_id = req.request_id
            
            ros_msg.start.x = int(req.start.x)
            ros_msg.start.y = int(req.start.y)
            ros_msg.goal.x = int(req.goal.x)
            ros_msg.goal.y = int(req.goal.y)

            if use_central:
                self.path_request_central_pub.publish(ros_msg)
            else:
                self.path_request_pub.publish(ros_msg)
                
        except Exception as exc:
            self.get_logger().error(f"Fehler beim Senden der PathRequest: {exc}")

    def _on_path_complete(self, msg: PathComplete) -> None:
        """Leitet die Antwort von ROS an den PathManager weiter."""
        try:
            from app.managers.path_manager import path_manager
            path_dict = message_to_ordereddict(msg)
            path_manager.handle_path_complete(path_dict)
        except Exception as exc:
            self.get_logger().error(f"Fehler in _on_path_complete: {exc}")

    def destroy(self) -> None:
        """Sicheres Herunterfahren der Node."""
        for t in self.heartbeat_timers.values():
            t.cancel()
            self.destroy_timer(t)
        self.heartbeat_timers.clear()
        super().destroy_node()


# ====================================================================== #
#                        Infrastruktur-API                               #
# ====================================================================== #
_ros_instance: Optional[RosClient] = None
_ros_executor: Optional[SingleThreadedExecutor] = None
_ros_thread: Optional[threading.Thread] = None

def start_ros_spin() -> RosClient:
    global _ros_instance, _ros_executor, _ros_thread

    if _ros_instance is not None:
        return _ros_instance

    if not rclpy.ok():
        rclpy.init()

    _ros_instance = RosClient()
    _ros_executor = SingleThreadedExecutor()
    _ros_executor.add_node(_ros_instance)

    def _spin():
        try:
            _ros_executor.spin()
        finally:
            if _ros_instance:
                _ros_instance.destroy()
            if rclpy.ok():
                rclpy.shutdown()

    _ros_thread = threading.Thread(target=_spin, daemon=True)
    _ros_thread.start()
    return _ros_instance

def get_ros_client() -> RosClient:
    return _ros_instance or start_ros_spin()

def restart_ros_client() -> RosClient:
    """Wird bei Modus-Wechsel (Sim/HW) aufgerufen."""
    global _ros_instance, _ros_executor, _ros_thread

    if _ros_executor:
        _ros_executor.shutdown()
    if _ros_instance:
        _ros_instance.destroy()
    if _ros_thread and _ros_thread.is_alive():
        _ros_thread.join(timeout=2.0)

    agent_manager.clear_all_agents()

    _ros_instance = None
    _ros_executor = None
    _ros_thread   = None

    return start_ros_spin()

# Automatischer Start beim Import
start_ros_spin()