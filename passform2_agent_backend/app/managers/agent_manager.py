import subprocess
import logging
from typing import Dict, List, Optional, Any

from passform_agent_msgs.msg import AgentInfo
from rosidl_runtime_py import message_to_ordereddict

from app.socket.socket_io_manager import socket_manager
from .persistence_manager import persistence_manager

logger = logging.getLogger("agent_manager")

class AgentEntry:
    """Bündelt alle Informationen eines Agenten (ROS-Status + Prozess-Handle)"""
    def __init__(self, agent_id: str, module_type: str):
        self.agent_id = agent_id
        self.module_type = module_type
        self.ros_info: Optional[Dict] = None
        self.process: Optional[subprocess.Popen] = None
        self.status = "initializing"  # Status: initializing, starting, active, offline

    def update_ros(self, agent_info: AgentInfo):
        """Aktualisiert die Daten basierend auf einem ROS-Heartbeat"""
        self.ros_info = message_to_ordereddict(agent_info)
        self.status = "active"

    def to_dict(self) -> Dict[str, Any]:
        """Erstellt ein Dictionary für das Frontend und die Persistenz"""
        if self.ros_info:
            data = dict(self.ros_info)
        else:
            data = {
                "agent_id": self.agent_id,
                "module_type": self.module_type,
                "position": {"x": 0, "y": 0},
                "orientation": 0.0
            }
        data["status"] = self.status
        return data


class AgentManager:
    """Zentrale Instanz zur Verwaltung aller aktiven Agenten (Singleton)"""
    
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AgentManager, cls).__new__(cls)
            cls._instance.agents: Dict[str, AgentEntry] = {}
            logger.info("AgentManager Singleton initialisiert.")
        return cls._instance

    def log_to_system(self, message: str, level: str = "info"):
        """Sendet eine Nachricht an das Elm-System-Log in der Sidebar"""
        logger.info(f"SystemLog: {message}")
        socket_manager.emit_event_sync('system_log', {
            "message": message,
            "level": level
        })

    def sync_from_ros(self, agent_info: AgentInfo):
        """Wird aufgerufen, wenn ein ROS-Heartbeat (AgentAnnounce) empfangen wird"""
        aid = agent_info.agent_id
        is_new = aid not in self.agents
        
        if is_new:
            self.agents[aid] = AgentEntry(aid, agent_info.module_type)
            self.log_to_system(f"Neuer Agent erkannt (ROS): {aid}", "success")
        
        # Falls der Agent vorher nur 'starting' war, jetzt auf 'active'
        was_starting = not is_new and self.agents[aid].status == "starting"
        
        self.agents[aid].update_ros(agent_info)
        
        if was_starting:
            self.log_to_system(f"Agent {aid} ist jetzt betriebsbereit.", "success")
            
        self.send_agent_list()

    def register_local_process(self, agent_id: str, module_type: str, process: subprocess.Popen):
        """Verknüpft einen lokal gestarteten Simulations-Prozess mit einem Agenten"""
        if agent_id not in self.agents:
            self.agents[agent_id] = AgentEntry(agent_id, module_type)
        
        self.agents[agent_id].process = process
        self.agents[agent_id].status = "starting"
        
        self.log_to_system(f"Simulations-Prozess gestartet: {agent_id} ({module_type})")
        self.send_agent_list()

    def remove_agent(self, agent_id: str):
        """Entfernt einen Agenten und beendet ggf. den zugehörigen Prozess"""
        if agent_id in self.agents:
            entry = self.agents.pop(agent_id)
            if entry.process and entry.process.poll() is None:
                entry.process.terminate()
                self.log_to_system(f"Prozess für {agent_id} beendet.", "warning")
            
            self.log_to_system(f"Agent entfernt: {agent_id}", "info")
            self.send_agent_list()

    def clear_all_agents(self):
        """Bereinigt das gesamte System"""
        if not self.agents:
            return
        self.log_to_system("Bereinige alle aktiven Agenten...", "warning")
        for aid in list(self.agents.keys()):
            self.remove_agent(aid)

    def send_agent_list(self):
        """Sendet aktuelle Liste an Elm und speichert sie persistent"""
        agents_data = [a.to_dict() for a in self.agents.values()]
        persistence_manager.save_state(agents_data)
        socket_manager.emit_event_sync('active_agents', {'agents': agents_data})

# Singleton Instanz
agent_manager = AgentManager()