# ros_agent_router.py
#
# FastAPI-Router, der beliebig viele ROS-Agenten parallel starten
# und beenden kann, ohne dass lange Locks alles blockieren.

import signal
import subprocess
import threading
import os
from typing import Dict, List, Tuple

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter()


# --------------------------------------------------------------------------- #
# Pydantic-Schemas für die REST-Endpunkte
# --------------------------------------------------------------------------- #

class StartAgentRequest(BaseModel):
    node_id: str
    position: List[int]           # [x, y]
    module_type: str


class KillAgentRequest(BaseModel):
    node_id: str


# --------------------------------------------------------------------------- #
# Node-Manager
# --------------------------------------------------------------------------- #

class NodeManager:
    """
    Verwaltet laufende ROS-Nodes.
    
    Simulation-Nodes laufen immer auf Domain ID 1.
    """

    def __init__(self) -> None:
        # node_id  ➞  Infos (Thread, Prozess, Position, ...)
        self.nodes: Dict[str, Dict] = {}

        # position (x, y)  ➞  node_id
        self.positions: Dict[Tuple[int, int], str] = {}

        self.global_lock = threading.RLock()

        print("NodeManager: Simulation-Nodes verwenden Domain ID 1")

    # ------------------------------------------------------------------ #
    # öffentliche API
    # ------------------------------------------------------------------ #

    def start_node(self, node_id: str,
                   position: Tuple[int, int],
                   module_type: str) -> None:
        """
        Startet einen neuen Simulations-Agenten (immer Domain ID 1).
        """

        position_str = str(list(position)).replace(" ", "")
        cmd = [
            "ros2", "launch", "passform_agent_planning",
            "single_agent.launch.py",
            f"position:={position_str}",
            f"module_type:={module_type}",
        ]

        # ------------------------------------------------------------
        # 1. Konflikte *identifizieren* (Lock nur Millisekunden)
        # ------------------------------------------------------------
        to_kill: List[str] = []
        with self.global_lock:
            old_id = self.positions.get(position)
            if old_id and old_id != node_id:
                to_kill.append(old_id)

            if node_id in self.nodes:
                to_kill.append(node_id)

        # ------------------------------------------------------------
        # 2. Konflikte *auflösen* (ohne Lock – kann dauern)
        # ------------------------------------------------------------
        for nid in set(to_kill):
            self._kill_node(nid)

        # ------------------------------------------------------------
        # 3. Node-Infos anlegen (Lock nur kurz)
        # ------------------------------------------------------------
        termination_event = threading.Event()
        with self.global_lock:
            self.nodes[node_id] = {
                "thread": None,
                "process": None,
                "termination_event": termination_event,
                "command": cmd,
                "position": position,
            }
            self.positions[position] = node_id

        # ------------------------------------------------------------
        # 4. separaten Thread für den Sub-Prozess starten
        # ------------------------------------------------------------
        def node_thread() -> None:
            try:
                # Simulation-Nodes immer auf Domain ID 1
                env = os.environ.copy()
                env['ROS_DOMAIN_ID'] = '1'
                
                proc = subprocess.Popen(cmd, env=env)
                with self.global_lock:
                    if node_id in self.nodes:
                        self.nodes[node_id]["process"] = proc
                proc.wait()
            except Exception as exc:
                print(f"[ERROR] Node '{node_id}': {exc}")
            finally:
                termination_event.set()
                # Aufräumen (Lock wieder nur kurz)
                with self.global_lock:
                    self.nodes.pop(node_id, None)
                    if self.positions.get(position) == node_id:
                        self.positions.pop(position, None)

        thread = threading.Thread(target=node_thread, daemon=True)
        with self.global_lock:
            if node_id in self.nodes:                    # sollte immer stimmen
                self.nodes[node_id]["thread"] = thread
        thread.start()

    def kill_node(self, node_id: str, wait_time: int = 3) -> None:
        """Öffentliche Kill-Methode (non-blocking für den Aufrufer)."""
        threading.Thread(
            target=self._kill_node, args=(node_id, wait_time), daemon=True
        ).start()

    def kill_all_nodes(self, wait_time: int = 3) -> None:
        """Beendet alle laufenden Nodes (parallele Threads)."""
        with self.global_lock:
            node_ids = list(self.nodes.keys())

        for nid in node_ids:
            self.kill_node(nid, wait_time=wait_time)

    # ------------------------------------------------------------------ #
    # interne Helfer
    # ------------------------------------------------------------------ #

    def _kill_node(self, node_id: str, wait_time: int = 3) -> None:
        """
        Beendet einen Node in zwei Phasen:

            1. Infos *atomar* aus den Dicts entfernen (global_lock kurz)
            2. Prozess/Thread killen (ohne Lock)
        """
        # ---------- Phase 1 (schnell) ----------
        with self.global_lock:
            info = self.nodes.pop(node_id, None)
            if not info:
                return                         # Node existiert nicht (mehr)

            pos = info.get("position")
            if pos and self.positions.get(pos) == node_id:
                self.positions.pop(pos, None)

        # ---------- Phase 2 (langsam) ----------
        proc: subprocess.Popen | None = info.get("process")
        thr:  threading.Thread | None = info.get("thread")

        if proc:
            try:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=wait_time)
            except Exception as exc:
                print(f"[ERROR] Kill '{node_id}': {exc}")

        if thr:
            thr.join(timeout=wait_time)


# --------------------------------------------------------------------------- #
# FastAPI-Router
# --------------------------------------------------------------------------- #

node_manager = NodeManager()


@router.post("/start_agent_node")
async def start_agent_node(req: StartAgentRequest):
    try:
        if len(req.position) != 2:
            raise ValueError("position must contain exactly 2 integers")

        pos_tuple = tuple(req.position)
        node_manager.start_node(req.node_id, pos_tuple, req.module_type)
        return {
            "success": True,
            "message": f"Started agent node '{req.node_id}'.",
        }
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@router.post("/kill_agent_node")
async def kill_agent_node(req: KillAgentRequest):
    try:
        node_manager.kill_node(req.node_id)
        return {
            "success": True,
            "message": f"Kill command accepted for node '{req.node_id}'.",
        }
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@router.post("/kill_all_nodes")
async def kill_all_nodes():
    try:
        node_manager.kill_all_nodes()
        return {
            "success": True,
            "message": "Kill command accepted for all nodes.",
        }
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc
