import signal
import subprocess
import threading
import os
import logging
import time
from typing import Dict, List, Tuple, Any
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from app.config import config

logger = logging.getLogger("node_manager")
router = APIRouter()

# --- Pydantic-Schemas für die API ---
class StartAgentRequest(BaseModel):
    agent_id: str
    x: int
    y: int
    module_type: str

# --- Node-Manager Singleton ---
class NodeManager:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(NodeManager, cls).__new__(cls)
            cls._instance.processes: Dict[str, subprocess.Popen] = {}
            cls._instance.lock = threading.RLock()
            logger.info("✅ NodeManager Orchestrator bereit.")
        return cls._instance

    def _get_project_root(self):
        """Ermittelt den absoluten Pfad zum passform2 Hauptverzeichnis."""
        return os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

    # --- SYSTEM NODES ---

    def start_system_nodes(self):
        """Startet zentrale ROS-Nodes wie den monitor_node / central_planner."""
        base_path = self._get_project_root()
        
        paths_to_try = [
            os.path.join(base_path, "passform2_ws/src/passform_agent_planning/passform_agent_planning/monitor_node.py"),
            os.path.join(base_path, "passform2_ws/src/passform_agent_planning/monitor_node.py")
        ]
        
        script_path = next((p for p in paths_to_try if os.path.exists(p)), None)
        
        if not script_path:
            logger.error(f"❌ Monitor-Script konnte an keinem Ort gefunden werden!")
            return

        def monitor_thread():
            try:
                env = os.environ.copy()
                env['ROS_DOMAIN_ID'] = str(config.get_domain_id())
                
                cmd = [
                    "python3", script_path, 
                    "--ros-args", 
                    "-p", f"heartbeat_hz:={config.heartbeat_hz}",
                    "-p", f"timeout_factor:={config.timeout_factor}",
                    "--log-level", "INFO"
                ]
                
                proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
                with self.lock:
                    self.processes["system_monitor"] = proc
                logger.info(f"🖥️ System-Monitor/Planner gestartet (Hz: {config.heartbeat_hz})")
                proc.wait()
            except Exception as e:
                logger.error(f"❌ Fehler im Monitor-Node: {e}")

        threading.Thread(target=monitor_thread, name="SystemMonitor", daemon=True).start()

    # --- AGENT NODES ---

    def start_agents_from_config(self, agents_list: List[Dict[str, Any]]):
        logger.info(f"🚀 Orchestriere {len(agents_list)} Agenten-Nodes...")
        for agent in agents_list:
            self.start_node(agent.get("agent_id"), (agent.get("x", 0), agent.get("y", 0)), agent.get("module_type", "unknown"))

    def start_node(self, agent_id: str, position: Tuple[int, int], module_type: str) -> None:
        """Startet einen einzelnen Agent-Node Prozess mit SSoT Parametern."""
        with self.lock:
            if agent_id in self.processes: return

        def node_thread():
            try:
                base_path = self._get_project_root()
                env = os.environ.copy()
                env['ROS_DOMAIN_ID'] = str(config.get_domain_id())
                
                paths_to_try = [
                    os.path.join(base_path, "passform2_ws/src/passform_agent_planning/passform_agent_planning/agent_node.py"),
                    os.path.join(base_path, "passform2_ws/src/passform_agent_planning/agent_node.py")
                ]
                script_path = next((p for p in paths_to_try if os.path.exists(p)), None)

                if not script_path:
                    logger.error(f"❌ Agent-Script für {agent_id} nicht gefunden.")
                    return

                cmd = [
                    "python3", script_path, 
                    "--ros-args", 
                    "-p", f"agent_id:={agent_id}", 
                    "-p", f"module_type:={module_type}", 
                    "-p", f"position:=[{position[0]},{position[1]}]",
                    "-p", f"heartbeat_hz:={config.heartbeat_hz}",
                    "--log-level", "INFO"
                ]
                
                proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
                with self.lock:
                    self.processes[agent_id] = proc
                proc.wait()
            except Exception as e:
                logger.error(f"❌ Fehler im Prozess von {agent_id}: {e}")
            finally:
                with self.lock:
                    if agent_id in self.processes: del self.processes[agent_id]

        threading.Thread(target=node_thread, name=f"Thread_{agent_id}", daemon=True).start()

    # --- RESTART LOGIK (NEU) ---

    def restart_active_agents(self):
        """
        Stoppt alle laufenden Agenten und startet sie neu.
        Wird genutzt, um globale Parameteränderungen (z.B. Herzrate) zu übernehmen.
        """
        with self.lock:
            # Kopie der IDs, um während der Iteration löschen/starten zu können
            active_agent_ids = [aid for aid in self.processes.keys() if aid != "system_monitor"]
        
        if not active_agent_ids:
            logger.info("ℹ️ Keine aktiven Agenten zum Neustarten vorhanden.")
            return

        logger.info(f"🔄 Starte {len(active_agent_ids)} Agenten neu (Synchronisation mit SSoT)...")

        # Lokaler Import um Circular Dependency zu vermeiden
        from .agent_manager import agent_manager

        for aid in active_agent_ids:
            agent = agent_manager.agents.get(aid)
            if agent:
                # 1. Prozess beenden
                self.kill_node(aid)
                # 2. Kurz warten (OS Cleanup)
                time.sleep(0.2)
                # 3. Mit neuen Parametern aus SSoT starten
                self.start_node(aid, (agent.x, agent.y), agent.module_type)
        
        logger.info("✅ Alle Agenten erfolgreich neu orchestriert.")

    # --- TERMINATION ---

    def kill_node(self, agent_id: str):
        with self.lock:
            proc = self.processes.get(agent_id)
            if proc:
                try:
                    # Sendet SIGTERM an die gesamte Prozessgruppe
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    logger.info(f"🛑 Node {agent_id} beendet.")
                except Exception as e:
                    logger.warning(f"⚠️ Konnte Node {agent_id} nicht sauber beenden: {e}")

    def kill_all_nodes(self):
        logger.info("🛑 Herunterfahren aller ROS-Nodes...")
        with self.lock:
            for aid in list(self.processes.keys()):
                self.kill_node(aid)

node_manager = NodeManager()

@router.post("/start_agent")
async def api_start_agent(req: StartAgentRequest):
    node_manager.start_node(req.agent_id, (req.x, req.y), req.module_type)
    return {"status": "ok"}