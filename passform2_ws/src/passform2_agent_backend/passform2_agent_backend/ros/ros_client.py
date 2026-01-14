#!/usr/bin/env python3
"""
Backend-Bridge zwischen Python-Web-App (FastAPI + Socket.IO) und ROS 2.

* Empfängt 10-Hz-Heartbeats (`AgentAnnounce`) aller Roboter-/Fördermodul-Agenten
  und hält eine Live-Liste aktiver Agenten.
* Leitet Path-Requests aus der Web-App als ROS-Nachrichten weiter.
* Gibt Path-Complete-Antworten wieder an die Web-App zurück.
"""

from __future__ import annotations

# ------------------------------------------------------ Standard-Bibliothek
import os
import threading
from typing import Dict, List, Optional, Callable, Any

# ------------------------------------------------------ ROS 2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.timer import Timer

from rosidl_runtime_py import message_to_ordereddict

from passform_agent_msgs.msg import (
    AgentAnnounce,
    AgentInfo,
    PathRequest,
    PathComplete,
)
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

# ------------------------------------------------------ Projekt-Eigene
from app.config import Config
from app.socket.socket_io_manager import socket_manager          # noqa: F401 (Seite Zukunft)
from app.managers.agent_manager import agent_manager
# Lazy-Import, um eine zirkuläre Abhängigkeit zu vermeiden:
path_manager: Optional[Any] = None
def get_path_manager():
    global path_manager
    if path_manager is None:
        from app.managers.path_manager import path_manager as pm
        path_manager = pm
    return path_manager


# ====================================================================== #
#                                  NODE                                  #
# ====================================================================== #
class RosClient(Node):
    """Zentrale ROS-Node der Web-Applikation."""

    # ------------------------------------------------------------------ #
    def __init__(self) -> None:
        # Domain ID setzen, **bevor** der Node angelegt wird
        os.environ["ROS_DOMAIN_ID"] = str(Config.get_domain_id())

        super().__init__("python_backend_node")
        self.get_logger().info(
            f"ROS Client gestartet – Mode: {Config.get_current_mode().value}, "
            f"Domain: {Config.get_domain_id()}"
        )

        # ----------------- Heartbeat-Parameter -----------------
        self.heartbeat_period: float = 1   # 10 Hz  -> 0,1 s
        self.timeout_factor:    int   = 3     # 3 × 0,1 s = 0,3 s

        # Laufende Heartbeat-Timer & bekannte Agenten
        self.known_agents: Dict[str, AgentInfo]           = {}
        self.heartbeat_timers: Dict[str, Timer]           = {}
        # -------------------------------------------------------

        self.current_request: Optional[str] = None
        self.request_info: Dict[str, Any]   = {}

        self._setup_ros_interfaces()

    # ------------------------------------------------------------------ #
    #                            ROS-Interfaces                           #
    # ------------------------------------------------------------------ #
    def _setup_ros_interfaces(self) -> None:
        """Publisher & Subscriber anlegen."""

        # # QoS: Agent-Announcements sind *transient-local* & *reliable*
        # ann_qos = QoSProfile(depth=10)
        # ann_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        # ann_qos.reliability = ReliabilityPolicy.RELIABLE

        self.agent_announce_sub = self.create_subscription(
            AgentAnnounce, "agent_announce", self._on_agent_announce, 1
        )

        self.path_request_pub = self.create_publisher(PathRequest, "path_request", 10)
        self.path_request_central_pub = self.create_publisher(PathRequest, "path_request_central", 10)

        self.path_complete_sub = self.create_subscription(
            PathComplete, "path_complete", self._on_path_complete, 10
        )

        self.get_logger().info("ROS-Interfaces eingerichtet")

    # ------------------------------------------------------------------ #
    #                       Heartbeat-Verarbeitung                       #
    # ------------------------------------------------------------------ #
    def _reset_heartbeat_timer(self, agent_info: AgentInfo) -> None:
        """(Re)initialisiert den Timer für einen Agenten-Heartbeat."""
        agent_id = agent_info.agent_id

        # vorherigen Timer (falls vorhanden) stoppen
        old = self.heartbeat_timers.pop(agent_id, None)
        if old:
            self.destroy_timer(old)
        else:
            # neuer Agent entdeckt
            self.known_agents[agent_id] = agent_info
            agent_manager.add_agent(agent_info)
            self.get_logger().info(f"Agent online: {agent_id}")

        # Callback, wenn kein Heartbeat mehr kommt
        def _on_timeout() -> None:
            if agent_id in self.known_agents:
                agent_manager.remove_agent(agent_id)
                del self.known_agents[agent_id]
                self.get_logger().warning(f"Agent offline (Timeout): {agent_id}")
            t = self.heartbeat_timers.pop(agent_id, None)
            if t:
                self.destroy_timer(t)

        timeout = self.timeout_factor * self.heartbeat_period  # 0,3 s
        self.heartbeat_timers[agent_id] = self.create_timer(timeout, _on_timeout)

    # ------------------------------------------------------------------ #
    def _on_agent_announce(self, msg: AgentAnnounce) -> None:
        """Jede eingehende Nachricht ist ein Heartbeat."""
        try:
            self._reset_heartbeat_timer(msg.agent_info)
        except Exception as exc:  # pragma: no-cover
            self.get_logger().error(f"Fehler in _on_agent_announce: {exc}")

    # ------------------------------------------------------------------ #
    #                        Path Request / Response                      #
    # ------------------------------------------------------------------ #
    def send_path_request(self, req, useCentral: bool = False) -> None:
        """Konvertiert & publiziert eine PathRequest‐Nachricht."""
        try:
            pr = PathRequest()
            pr.request_id = req.request_id
            pr.start.x, pr.start.y = req.start.x, req.start.y
            pr.goal.x,  pr.goal.y  = req.goal.x,  req.goal.y
            if(useCentral):
                print("send central request")
                self.path_request_central_pub.publish(pr)
            else:
                self.path_request_pub.publish(pr)
            self.get_logger().info(f"PathRequest gesendet: {req.request_id}")
        except Exception as exc:  # pragma: no-cover
            self.get_logger().error(f"Fehler beim Senden der PathRequest: {exc}")

    # ------------------------------------------------------------------ #
    def _on_path_complete(self, msg: PathComplete) -> None:
        """Antwort des Agentenverbunds → an die Web-App durchreichen."""
        try:
            pm = get_path_manager()
            pm.add_response(message_to_ordereddict(msg))
        except Exception as exc:  # pragma: no-cover
            self.get_logger().error(f"Fehler in _on_path_complete: {exc}")

    # ------------------------------------------------------------------ #
    def get_all_agents(self) -> List[AgentInfo]:
        """Liefert aktuelle Agentenliste (ohne Timeout)."""
        return list(self.known_agents.values())

    # ------------------------------------------------------------------ #
    def destroy(self) -> None:
        """Aufräumen (Timer stoppen), dann Node zerlegen."""
        for t in self.heartbeat_timers.values():
            self.destroy_timer(t)
        self.heartbeat_timers.clear()
        super().destroy_node()


# ====================================================================== #
#                         Hilfsfunktionen / API                          #
# ====================================================================== #
_ros_instance: Optional[RosClient]      = None
_ros_executor: Optional[SingleThreadedExecutor] = None
_ros_thread: Optional[threading.Thread] = None


def start_ros_spin() -> RosClient:
    """Startet (oder liefert) den globalen RosClient + Spin-Thread."""
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
    """Stellt sicher, dass immer ein lebender RosClient vorhanden ist."""
    return _ros_instance or start_ros_spin()


def restart_ros_client() -> RosClient:
    """Stoppt Node & Executor und startet sie mit neuer Domain neu."""
    global _ros_instance, _ros_executor, _ros_thread

    print(f"🔄 Starte ROS Client neu – neue Domain: {Config.get_domain_id()}")

    # alten Client/Executor herunterfahren
    if _ros_executor:
        _ros_executor.shutdown()
    if _ros_instance:
        _ros_instance.destroy()
    if _ros_thread and _ros_thread.is_alive():
        _ros_thread.join(timeout=2.0)

    agent_manager.clear_all_agents()

    os.environ["ROS_DOMAIN_ID"] = str(Config.get_domain_id())

    _ros_instance = None
    _ros_executor = None
    _ros_thread   = None

    return start_ros_spin()


# ---------------------------------------------------------------------- #
# Optional: Node sofort beim Import starten
start_ros_spin()
