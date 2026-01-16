import subprocess
import logging
import asyncio
from typing import Dict, List, Optional, Any, Tuple

from passform_agent_msgs.msg import AgentInfo
from rosidl_runtime_py import message_to_ordereddict

from app.socket.socket_io_manager import socket_manager
from .persistence_manager import persistence_manager
from app.config import config, SystemMode

logger = logging.getLogger("agent_manager")

class AgentEntry:
    def __init__(self, agent_id: str, module_type: str, x: int = 0, y: int = 0):
        self.agent_id = agent_id
        self.module_type = module_type
        self.x = x
        self.y = y
        self.orientation = 0.0
        self.status = "idle"
        self.is_dynamic = (module_type.lower() == "ftf")
        self.payload: Optional[Dict] = None # Jetzt speichern wir das ganze Agent-Dict als Payload
        self.process: Optional[subprocess.Popen] = None

    def update_from_ros(self, agent_info: AgentInfo):
        self.x = int(agent_info.position.x)
        self.y = int(agent_info.position.y)
        self.orientation = float(agent_info.orientation)
        self.status = "active"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "agent_id": str(self.agent_id),
            "module_type": str(self.module_type),
            "is_dynamic": self.is_dynamic,
            "payload": self.payload.get("agent_id") if self.payload else None,
            "position": {"x": int(self.x), "y": int(self.y)},
            "x": int(self.x),
            "y": int(self.y),
            "orientation": int(self.orientation),
            "status": str(self.status)
        }

class AgentManager:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AgentManager, cls).__new__(cls)
            cls._instance.agents: Dict[str, AgentEntry] = {}
            cls._instance.logs_history: List[Dict[str, str]] = []
            cls._instance.max_logs = 30
        return cls._instance

    # --- MISSIONS-ABARBEITUNG ---
    async def execute_mission(self, ftf_id: str, full_path: List[Dict], start_pos: Tuple[int, int], goal_pos: Tuple[int, int]):
        """Simuliert die Fahrt des FTF und das Handling des Moduls."""
        ftf = self.agents.get(ftf_id)
        if not ftf: return

        self.log_to_system(f"FTF {ftf_id} startet Mission...", "info")

        for i, node in enumerate(full_path):
            # 1. Position aktualisieren
            ftf.x = node.get("x")
            ftf.y = node.get("y")
            ftf.status = "moving"
            
            # 2. Pickup-Logik: Wenn wir am Startpunkt ankommen
            if (ftf.x, ftf.y) == start_pos and not ftf.payload:
                # Wir suchen das statische Modul an dieser Stelle
                target_agent_id = None
                for aid, a in self.agents.items():
                    if not a.is_dynamic and (a.x, a.y) == start_pos:
                        target_agent_id = aid
                        break
                
                if target_agent_id:
                    self.log_to_system(f"📦 Modul {target_agent_id} aufgenommen.", "success")
                    ftf.payload = self.agents.pop(target_agent_id).to_dict()
            
            # 3. Drop-Logik: Wenn wir am Zielpunkt ankommen
            if (ftf.x, ftf.y) == goal_pos and ftf.payload:
                payload_data = ftf.payload
                new_id = payload_data["agent_id"]
                self.agents[new_id] = AgentEntry(new_id, payload_data["module_type"], ftf.x, ftf.y)
                ftf.payload = None
                self.log_to_system(f"✅ Modul an Ziel {goal_pos} abgesetzt.", "success")

            # Update an alle Clients senden
            self.send_agent_list()
            await asyncio.sleep(0.8) # Geschwindigkeit der Simulation

        ftf.status = "idle"
        self.send_agent_list()
        self.log_to_system("Mission abgeschlossen.", "success")

    def get_grid_model(self) -> Dict[Tuple[int, int], Dict]:
        return {(a.x, a.y): a.to_dict() for a in self.agents.values()}

    def send_agent_list(self):
        agents_data = [a.to_dict() for a in self.agents.values()]
        socket_manager.emit_event_sync('active_agents', {'agents': agents_data})

    def log_to_system(self, message: str, level: str = "info"):
        log_entry = {"message": message, "level": level}
        self.logs_history.append(log_entry)
        socket_manager.emit_event_sync('system_log', log_entry)

    def get_logs_history(self): return self.logs_history

    def clear_all_agents(self):
        self.agents.clear()
        self.send_agent_list()

agent_manager = AgentManager()