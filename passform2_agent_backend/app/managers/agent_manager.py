import logging
import asyncio
import time
from datetime import datetime
from typing import Dict, List, Optional, Any, Tuple

# Relative Imports
from .socket_io_manager import socket_manager
from .config_manager import config_manager
from app.config import config  # Zugriff auf HEARTBEAT_HZ und TIMEOUT_FACTOR

logger = logging.getLogger("agent_manager")

class AgentEntry:
    def __init__(self, agent_id: str, module_type: str, x: int = 0, y: int = 0, orientation: int = 0):
        self.agent_id = agent_id
        self.module_type = module_type
        self.x = x
        self.y = y
        self.orientation = orientation
        self.status = "active"
        self.is_dynamic = (module_type.lower() == "ftf")
        self.payload: Optional[Dict] = None 
        
        # Signalstärken-Management
        self.last_seen = time.time()

    def update_from_ros(self, x: int, y: int, orientation: int):
        """Wird vom ROS-Client aufgerufen, um Live-Daten zu übernehmen."""
        self.x = x
        self.y = y
        self.orientation = orientation
        self.status = "active"
        self.last_seen = time.time()

    def get_signal_strength(self) -> int:
        """Berechnet die Signalstärke basierend auf dem SSoT-Timeout."""
        dt = time.time() - self.last_seen
        # T_timeout = (1/f) * factor
        timeout = config.heartbeat_period * config.timeout_factor
        strength = max(0, int(100 * (1 - dt / timeout)))
        return strength

    def to_dict(self) -> Dict[str, Any]:
        return {
            "agent_id": str(self.agent_id),
            "module_type": str(self.module_type),
            "is_dynamic": self.is_dynamic,
            "payload": self.payload, # Enthält das Dict des transportierten Moduls
            "x": int(self.x),
            "y": int(self.y),
            "orientation": int(self.orientation),
            "status": str(self.status),
            "signal_strength": self.get_signal_strength()
        }

class AgentManager:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AgentManager, cls).__new__(cls)
            cls._instance.agents: Dict[str, AgentEntry] = {}
            cls._instance.logs_history = []
            cls._instance.max_logs = 50
        return cls._instance

    # --- AGENTEN VERWALTUNG ---

    def add_agent(self, agent_data: Dict):
        agent_id = agent_data.get("agent_id")
        if not agent_id: return

        new_agent = AgentEntry(
            agent_id=agent_id,
            module_type=agent_data.get("module_type", "static"),
            x=agent_data.get("x", 0),
            y=agent_data.get("y", 0),
            orientation=agent_data.get("orientation", 0)
        )
        self.agents[agent_id] = new_agent
        self.save_to_ssot()

        # ROS-Node via NodeManager starten
        from .node_manager import node_manager
        node_manager.start_node(
            agent_id, 
            (new_agent.x, new_agent.y), 
            new_agent.module_type
        )

        self.log_to_system(f"🚀 Modul {agent_id} erstellt.", "success")
        self.send_agent_list()

    def remove_agent(self, agent_id: str):
        if agent_id in self.agents:
            from .node_manager import node_manager
            node_manager.kill_node(agent_id)

            del self.agents[agent_id]
            self.save_to_ssot()
            self.log_to_system(f"🗑 Modul {agent_id} entfernt.", "warning")
            self.send_agent_list()

    # --- SSoT LOGIK ---

    def load_from_ssot(self):
        data = config_manager.load_config()
        agent_list = data.get("agents", [])
        
        self.agents.clear()
        for a in agent_list:
            x = a.get("x", 0)
            y = a.get("y", 0)
            
            self.agents[a["agent_id"]] = AgentEntry(
                agent_id=a["agent_id"],
                module_type=a["module_type"],
                x=int(x),
                y=int(y),
                orientation=a.get("orientation", 0)
            )
        logger.info(f"✅ {len(self.agents)} Agenten aus SSoT geladen.")

    def save_to_ssot(self):
        config_manager.current_data["agents"] = [a.to_dict() for a in self.agents.values()]
        config_manager.current_data["last_update"] = datetime.now().isoformat()
        config_manager.save_to_ssot()

    # --- MISSIONS-LOGIK (Transport & Animation) ---

    async def execute_mission(self, ftf_id: str, full_path: List[Dict], start_pos: Tuple[int, int], goal_pos: Tuple[int, int]):
        ftf = self.agents.get(ftf_id)
        if not ftf: return

        self.log_to_system(f"FTF {ftf_id} führt Mission aus...", "info")

        for node in full_path:
            ftf.x, ftf.y = node.get("x"), node.get("y")
            ftf.status = "moving"
            ftf.last_seen = time.time() # In Simulation "Sichtkontakt" halten
            
            # Pickup-Logik
            if (ftf.x, ftf.y) == start_pos and not ftf.payload:
                target_id = next((aid for aid, a in self.agents.items() 
                                 if not a.is_dynamic and (a.x, a.y) == start_pos), None)
                if target_id:
                    self.log_to_system(f"📦 Modul {target_id} aufgenommen.", "success")
                    target_agent = self.agents.pop(target_id)
                    ftf.payload = target_agent.to_dict()
            
            # Drop-Logik
            if (ftf.x, ftf.y) == goal_pos and ftf.payload:
                p = ftf.payload
                self.agents[p["agent_id"]] = AgentEntry(p["agent_id"], p["module_type"], ftf.x, ftf.y, p.get("orientation", 0))
                ftf.payload = None
                self.log_to_system(f"✅ Modul abgesetzt.", "success")

            self.send_agent_list()
            await asyncio.sleep(0.5)

        ftf.status = "active"
        self.send_agent_list()
        self.save_to_ssot()

    # --- BROADCAST & HEALTH ---

    def send_agent_list(self):
        """
        Pusht die Agenten ans Frontend. 
        WICHTIG: Wir nutzen jetzt das gleiche Format wie die ROS-Bridge ("x,y": Daten),
        damit der Elm-Decoder für beide Ports funktioniert.
        """
        # Wir bauen ein Dictionary mit "x,y" als Keys
        agents_dict = {}
        for a in self.agents.values():
            key = f"{a.x},{a.y}"
            agents_dict[key] = a.to_dict()

        # Wir senden direkt das Dict (ohne den Umweg über den 'agents' Key),
        # genau so wie es die socket_bridge.py auf Port 5000 macht.
        socket_manager.emit_event_sync('active_agents', agents_dict)

    def get_logs_history(self):
        """Gibt die echten gesammelten Logs zurück."""
        return self.logs_history

    async def start_health_broadcast_loop(self):
        """Ein Hintergrund-Task, der die Signalstärke im UI flüssig hält."""
        while True:
            self.send_agent_list()
            await asyncio.sleep(2.0) # Alle 2 Sek reicht für die Anzeige

    def log_to_system(self, message: str, level: str = "info"):
        log_entry = {"message": message, "level": level, "timestamp": datetime.now().strftime("%H:%M:%S")}
        self.logs_history.append(log_entry)
        if len(self.logs_history) > self.max_logs:
            self.logs_history.pop(0)
        socket_manager.emit_event_sync('system_log', log_entry)

    def sync_from_ros(self, msg):
        """Update-Schnittstelle für den RosClient."""
        agent_id = msg.agent_id
        if agent_id not in self.agents:
            # Falls unbekannter Agent (z.B. Hardware wird angesteckt)
            self.agents[agent_id] = AgentEntry(
                agent_id=agent_id, module_type=msg.module_type,
                x=int(msg.position.x), y=int(msg.position.y)
            )
        
        self.agents[agent_id].update_from_ros(
            x=int(msg.position.x), 
            y=int(msg.position.y), 
            orientation=int(getattr(msg, 'orientation', 0))
        )
        self.send_agent_list()

    def clear_all_agents(self):
        from .node_manager import node_manager
        node_manager.kill_all_nodes()
        self.agents.clear()
        self.save_to_ssot()
        self.send_agent_list()

agent_manager = AgentManager()