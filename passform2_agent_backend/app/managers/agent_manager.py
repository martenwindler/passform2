import subprocess
import logging
from typing import Dict, List, Optional, Any, Tuple

from passform_agent_msgs.msg import AgentInfo
from rosidl_runtime_py import message_to_ordereddict

from app.socket.socket_io_manager import socket_manager
from .persistence_manager import persistence_manager
from app.config import config, SystemMode

logger = logging.getLogger("agent_manager")

class AgentEntry:
    """Bündelt alle Informationen eines Agenten (ROS-Status + Prozess-Handle)."""
    
    def __init__(self, agent_id: str, module_type: str, x: int = 0, y: int = 0):
        self.agent_id = agent_id
        self.module_type = module_type
        self.x = x
        self.y = y
        self.orientation = 0.0
        self.status = "initializing"
        self.process: Optional[subprocess.Popen] = None
        self.ros_info: Optional[Dict] = None

    def update_from_ros(self, agent_info: AgentInfo):
        """Extrahiert Daten aus der ROS-Struktur."""
        try:
            self.ros_info = message_to_ordereddict(agent_info)
            self.x = int(agent_info.position.x)
            self.y = int(agent_info.position.y)
            self.orientation = float(agent_info.orientation)
            self.status = "active"
            logger.info(f"📡 Agent {self.agent_id} updated: ({self.x},{self.y}) @ {self.orientation}°")
        except AttributeError as e:
            logger.error(f"Strukturfehler in ROS-Message für {self.agent_id}: {e}")

    def update_manually(self, data: Dict[str, Any]):
        """Update für Simulation / manuelle Steuerung."""
        self.x = int(data.get("x", self.x))
        self.y = int(data.get("y", self.y))
        self.module_type = data.get("module_type", self.module_type)
        self.orientation = float(data.get("orientation", self.orientation))
        self.status = data.get("status", "simulated")

    def to_dict(self) -> Dict[str, Any]:
        """Erzeugt das flache JSON für Decoders.elm."""
        return {
            "agent_id": str(self.agent_id),
            "module_type": str(self.module_type),
            "x": int(self.x),
            "y": int(self.y),
            "orientation": int(self.orientation),
            "status": str(self.status)
        }


class AgentManager:
    """Zentrale Instanz zur Verwaltung aller aktiven Agenten (Singleton)."""
    
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AgentManager, cls).__new__(cls)
            cls._instance.agents: Dict[str, AgentEntry] = {}
            cls._instance.logs_history: List[Dict[str, str]] = []
            cls._instance.max_logs = 30
            logger.info("AgentManager Singleton initialisiert.")
        return cls._instance

    def sync_from_ros(self, agent_info: AgentInfo):
        """Zentraler Einstiegspunkt für ROS-Heartbeats."""
        aid = agent_info.agent_id
        
        if aid not in self.agents:
            self.agents[aid] = AgentEntry(aid, agent_info.module_type)
            self.log_to_system(f"Neuer Agent erkannt: {aid}", "success")

        self.agents[aid].update_from_ros(agent_info)
        self.send_agent_list()

    def update_agent_manually(self, agent_id: str, data: Dict[str, Any]):
        """Schnittstelle für Simulator / API-Updates."""
        if agent_id not in self.agents:
            self.agents[agent_id] = AgentEntry(
                agent_id, 
                data.get("module_type", "unknown"),
                x=data.get("x", 0),
                y=data.get("y", 0)
            )
        
        self.agents[agent_id].update_manually(data)
        self.send_agent_list()

    def get_grid_model(self) -> Dict[Tuple[int, int], Dict]:
        """Gibt das aktuelle Weltmodell als Koordinaten-Dict zurück."""
        return {(a.x, a.y): a.to_dict() for a in self.agents.values()}

    def send_agent_list(self):
        """Synchronisiert Backend-Status mit Persistence und Frontend."""
        current_mode = config.get_current_mode()
        agents_data = [a.to_dict() for a in self.agents.values()]
        
        # Per WebSocket an Elm senden
        socket_manager.emit_event_sync('active_agents', {'agents': agents_data})
        
        # Nur im Hardware-Modus persistent speichern (Verhindert Ghosting in Simulation)
        if current_mode == SystemMode.HARDWARE:
            persistence_manager.save_state(agents_data)

    def remove_agent(self, agent_id: str):
        """Entfernt einen Agenten und beendet ggf. den Prozess."""
        if agent_id in self.agents:
            entry = self.agents.pop(agent_id)
            if entry.process and entry.process.poll() is None:
                entry.process.terminate()
            self.send_agent_list()

    def clear_all_agents(self):
        """Löscht alle Agenten (Reset für Hardware-Wechsel)."""
        self.agents.clear()
        logger.info("Sämtliche Agenten aus dem Manager entfernt.")
        self.send_agent_list()

    def log_to_system(self, message: str, level: str = "info"):
        """Speichert Logs im Buffer und sendet sie an Elm."""
        log_entry = {"message": message, "level": level}
        self.logs_history.append(log_entry)
        
        if len(self.logs_history) > self.max_logs:
            self.logs_history.pop(0)
            
        socket_manager.emit_event_sync('system_log', log_entry)

    def get_logs_history(self) -> List[Dict[str, str]]:
        """Gibt die gespeicherten Logs zurück."""
        return self.logs_history

# Singleton Instanz
agent_manager = AgentManager()