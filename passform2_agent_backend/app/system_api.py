import logging
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Literal

from .config import Config, SystemMode
from .managers.agent_manager import agent_manager
from .ros.ros_client import restart_ros_client
from app.socket.socket_io_manager import socket_manager

# Logger für die API
logger = logging.getLogger("system_api")
router = APIRouter()

# --- SCHEMAS ---
class ModeRequest(BaseModel):
    mode: Literal["hardware", "simulation"]

class RFIDRequest(BaseModel):
    rfid_id: str

# --- ENDPUNKTE ---

@router.get("/status")
async def get_system_status():
    """Gibt den aktuellen Systemzustand zurück"""
    return {
        "mode": Config.get_current_mode().value,
        "domain_id": Config.get_domain_id(),
        "active_agents": len(agent_manager.agents),
        "ros_ok": True  # Hier könnte man noch eine echte Prüfung einbauen
    }

@router.post("/mode")
async def switch_system_mode(request: ModeRequest):
    """Wechselt zwischen Hardware- und Simulationsmodus"""
    try:
        new_mode = SystemMode.HARDWARE if request.mode == "hardware" else SystemMode.SIMULATION
        
        # Nur wechseln, wenn sich der Modus wirklich ändert
        if not Config.set_mode(new_mode):
            return {"message": f"Bereits im {request.mode} Modus", "mode": request.mode}

        # --- REAKTION AUF WECHSEL ---
        logger.info(f"🔄 System-Reset wegen Modus-Wechsel auf {request.mode}")
        
        # 1. Alle Agenten und Prozesse stoppen
        agent_manager.clear_all_agents()
        
        # 2. ROS Client mit neuer Domain ID (0 oder 1) neu starten
        restart_ros_client()

        # 3. Elm-Clients informieren
        socket_manager.emit_event_sync('mode', request.mode)
        socket_manager.emit_event_sync('system_log', f"Modus gewechselt zu: {request.mode.upper()}")

        return {
            "message": f"Erfolgreich zu {request.mode} gewechselt",
            "mode": request.mode,
            "domain_id": Config.get_domain_id()
        }
        
    except Exception as e:
        logger.error(f"Fehler beim Modus-Wechsel: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/rfid_scan")
async def rfid_scan(req: RFIDRequest):
    """
    Endpunkt für Raspberry Pi Hardware.
    Empfängt RFID-Scans und leitet sie an Elm weiter.
    """
    try:
        logger.info(f"📟 RFID-Tag gescannt: {req.rfid_id}")
        
        # 1. Sofortige Rückmeldung an alle Elm-Frontends
        socket_manager.emit_event_sync('rfid_scanned', {'rfid_id': req.rfid_id})
        
        # 2. Eintrag in das System-Log in der Sidebar
        socket_manager.emit_event_sync('system_log', f"RFID-Chip erkannt: {req.rfid_id}")
        
        return {"status": "success", "id": req.rfid_id}
    except Exception as e:
        logger.error(f"RFID Fehler: {e}")
        raise HTTPException(status_code=500, detail="RFID-Verarbeitung fehlgeschlagen")