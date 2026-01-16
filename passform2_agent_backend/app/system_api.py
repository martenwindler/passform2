import logging
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Literal, List, Optional

# System-Importe
from .config import config, SystemMode
from .managers.agent_manager import agent_manager
from .managers.socket_io_manager import socket_manager
from .ros.ros_client import restart_ros_client

# Logger Setup
logger = logging.getLogger("system_api")
router = APIRouter()

# --- SCHEMAS ---

class ModeRequest(BaseModel):
    mode: Literal["hardware", "simulation"]

class RFIDRequest(BaseModel):
    rfid_id: str

class AgentNodeSchema(BaseModel):
    """Schema für die Orchestrierung neuer Module"""
    agent_id: str
    module_type: str
    x: int
    y: int
    orientation: Optional[int] = 0

# --- SYSTEM STEUERUNG (ALT) ---

@router.get("/status")
async def get_system_status():
    """Gibt den aktuellen Systemzustand zurück"""
    return {
        "mode": config.get_current_mode().value,
        "domain_id": config.get_domain_id(),
        "active_agents": len(agent_manager.agents),
        "nfc_status": "online" if hasattr(agent_manager, 'nfc_online') else "unknown",
        "ros_ok": True
    }

@router.post("/mode")
async def switch_system_mode(request: ModeRequest):
    """Wechselt den Modus, killt alle Nodes und startet ROS neu"""
    try:
        new_mode = SystemMode.HARDWARE if request.mode == "hardware" else SystemMode.SIMULATION
        
        if not config.set_mode(new_mode):
            return {"message": f"Bereits im {request.mode} Modus", "mode": request.mode}

        logger.info(f"🔄 System-Reset: Modus-Wechsel auf {request.mode}")
        
        # 1. Orchestrator-Reset: Stoppt Agenten UND deren ROS-Prozesse
        agent_manager.clear_all_agents()
        
        # 2. ROS Client mit neuer Domain ID neu starten
        restart_ros_client()

        # 3. Frontend synchronisieren
        socket_manager.emit_event_sync('mode', request.mode)
        socket_manager.emit_event_sync('system_log', {
            "message": f"SYSTEM-MODUS GEÄNDERT: {request.mode.upper()}",
            "level": "warning"
        })

        return {
            "message": f"Erfolgreich zu {request.mode} gewechselt",
            "mode": request.mode,
            "domain_id": config.get_domain_id()
        }
    except Exception as e:
        logger.error(f"Fehler beim Modus-Wechsel: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/rfid_scan")
async def rfid_scan(req: RFIDRequest):
    """Schnittstelle für externe RFID-Reader Hardware"""
    try:
        logger.info(f"📟 Externer RFID-Scan: {req.rfid_id}")
        socket_manager.emit_event_sync('rfid_found', req.rfid_id)
        socket_manager.emit_event_sync('system_log', {
            "message": f"RFID-Tag erkannt: {req.rfid_id}",
            "level": "info"
        })
        return {"status": "success", "id": req.rfid_id}
    except Exception as e:
        logger.error(f"RFID API Fehler: {e}")
        raise HTTPException(status_code=500, detail="RFID-Verarbeitung fehlgeschlagen")

# --- ORCHESTRIERUNG (NEU) ---

@router.post("/add_node")
async def add_node(node: AgentNodeSchema):
    """Fügt Modul hinzu, speichert in SSoT und startet ROS-Prozess"""
    try:
        agent_data = node.dict()
        agent_manager.add_agent(agent_data)
        return {"status": "success", "agent_id": node.agent_id}
    except Exception as e:
        logger.error(f"Fehler bei add_node API: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.delete("/remove_node/{agent_id}")
async def remove_node(agent_id: str):
    """Entfernt Modul aus SSoT und beendet ROS-Prozess"""
    try:
        agent_manager.remove_agent(agent_id)
        return {"status": "success", "removed": agent_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/logs")
async def get_logs():
    """Gibt die Log-Historie für die Sidebar zurück"""
    return {"logs": agent_manager.get_logs_history()}