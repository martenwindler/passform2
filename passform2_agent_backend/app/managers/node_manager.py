import signal
import subprocess
import threading
import os
import logging
from typing import Dict, List, Tuple
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from app.config import config
from app.managers.agent_manager import agent_manager

# Logger Setup
logger = logging.getLogger("node_manager")
router = APIRouter()

# --- Pydantic-Schemas für die API ---
class StartAgentRequest(BaseModel):
    node_id: str
    position: List[int]  # [x, y]
    module_type: str

class KillAgentRequest(BaseModel):
    node_id: str


# --- Node-Manager Singleton ---
class NodeManager:
    """
    Verwaltet die Betriebssystem-Prozesse der ROS 2 Simulation.
    Stellt sicher, dass Simulations-Nodes auf Domain ID 1 laufen.
    """
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(NodeManager, cls).__new__(cls)
            cls._instance.global_lock = threading.RLock()
            logger.info("NodeManager Singleton erfolgreich initialisiert.")
        return cls._instance

    def start_node(self, node_id: str, position: Tuple[int, int], module_type: str) -> None:
        """
        Startet einen Simulations-Agenten (ROS 2 Launch) in einem separaten Thread.
        """
        position_str = str(list(position)).replace(" ", "")
        
        # Befehl aus der zentralen Config beziehen
        cmd = config.get_launch_command(node_id, position_str, module_type)

        def node_thread() -> None:
            try:
                # Umgebungsvariablen für ROS Domain ID 1 (Simulation)
                env = os.environ.copy()
                env['ROS_DOMAIN_ID'] = str(config.settings.domain_ids['simulation'] if hasattr(config.settings, 'domain_ids') else '1')
                
                logger.info(f"Starte Prozess für Agent '{node_id}' mit Befehl: {' '.join(cmd)}")
                
                # Subprozess starten
                proc = subprocess.Popen(
                    cmd, 
                    env=env, 
                    preexec_fn=os.setsid  # Ermöglicht das Killen der gesamten Prozessgruppe
                )
                
                # Den Prozess im AgentManager registrieren (für Status-Updates an Elm)
                agent_manager.register_local_process(node_id, module_type, proc)
                
                # Blockiert den Thread, bis der Prozess endet
                proc.wait()
                
            except Exception as exc:
                logger.error(f"[ERROR] Fehler im Thread von Agent '{node_id}': {exc}")
            finally:
                # Wenn der Prozess endet (gekillt oder abgestürzt), im AgentManager aufräumen
                logger.info(f"Prozess für Agent '{node_id}' wurde beendet.")
                agent_manager.remove_agent(node_id)

        # Thread als Daemon starten, damit er das Backend beim Beenden nicht blockiert
        thread = threading.Thread(target=node_thread, name=f"AgentThread_{node_id}", daemon=True)
        thread.start()

    def kill_node(self, node_id: str) -> None:
        """Beendet einen spezifischen Agenten über den AgentManager."""
        agent_manager.remove_agent(node_id)

    def kill_all_nodes(self) -> None:
        """Beendet alle aktuell registrierten Agenten-Prozesse."""
        logger.info("Fahre alle Simulations-Nodes herunter...")
        agent_manager.clear_all_agents()


# Globale Singleton-Instanz erzeugen
node_manager = NodeManager()

# --- FastAPI Router Endpunkte ---

@router.post("/start_agent_node")
async def start_agent_node(req: StartAgentRequest):
    try:
        if len(req.position) != 2:
            raise ValueError("Position muss exakt 2 Werte enthalten [x, y]")
        
        pos_tuple = (req.position[0], req.position[1])
        node_manager.start_node(req.node_id, pos_tuple, req.module_type)
        
        return {
            "success": True,
            "message": f"Start-Befehl für Agent '{req.node_id}' gesendet."
        }
    except Exception as exc:
        logger.error(f"API Error (start_agent_node): {exc}")
        raise HTTPException(status_code=500, detail=str(exc))

@router.post("/kill_agent_node")
async def kill_agent_node(req: KillAgentRequest):
    try:
        node_manager.kill_node(req.node_id)
        return {
            "success": True,
            "message": f"Kill-Befehl für Agent '{req.node_id}' akzeptiert."
        }
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))

@router.post("/kill_all_nodes")
async def kill_all_nodes_api():
    try:
        node_manager.kill_all_nodes()
        return {
            "success": True,
            "message": "Alle laufenden Agenten-Prozesse werden beendet."
        }
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))