import logging
from typing import Dict, Any, Set
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from app.socket.socket_io_manager import socket_manager

# Logger Setup
logger = logging.getLogger("path_manager")
router = APIRouter()

# --- Pydantic-Schemas für die API ---
class Position(BaseModel):
    x: int
    y: int

class PlanPathRequest(BaseModel):
    request_id: str
    start: Position
    goal: Position
    central: bool = False  # Falls ein zentraler Koordinator genutzt werden soll


# --- Path-Manager Singleton ---
class PathManager:
    """
    Koordiniert Pfadanfragen zwischen Web-UI und ROS.
    Hält eine Liste ausstehender (pending) Requests, um Antworten zuzuordnen.
    """
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(PathManager, cls).__new__(cls)
            # Set für schnelle Suche nach ausstehenden IDs
            cls._instance.pending_requests: Set[str] = set()
            logger.info("PathManager Singleton erfolgreich initialisiert.")
        return cls._instance

    def request_path(self, req: PlanPathRequest) -> bool:
        """
        Registriert eine neue Anfrage und leitet sie an den ROS-Client weiter.
        """
        try:
            # Lazy Import, um zirkuläre Abhängigkeiten beim Start zu vermeiden
            from app.ros.ros_client import get_ros_client
            ros_client = get_ros_client()
            
            # ID registrieren
            self.pending_requests.add(req.request_id)
            
            # Über ROS publizieren
            ros_client.send_path_request(req, use_central=req.central)
            
            logger.info(f"Pfadplanung angefordert: ID={req.request_id} ({req.start} -> {req.goal})")
            return True
            
        except Exception as e:
            logger.error(f"Fehler bei Pfadanfrage {req.request_id}: {e}")
            self.pending_requests.discard(req.request_id)
            return False

    def handle_path_complete(self, path_data: Dict[str, Any]):
        """
        Verarbeitet die Antwort vom ROS-Client und sendet sie an Elm.
        Wird direkt vom ROS-Callback aufgerufen.
        """
        req_id = path_data.get('request_id')
        
        if req_id in self.pending_requests:
            logger.info(f"✅ Pfadplanung abgeschlossen für ID: {req_id}")
            self.pending_requests.discard(req_id)
        else:
            # Sicherheits-Fallback: Auch unbekannte IDs an Elm senden, 
            # falls Elm die ID noch kennt (z.B. nach Backend-Neustart)
            logger.warning(f"⚠️ Empfange Pfad für nicht registrierte ID: {req_id}")

        # Über WebSockets an Elm pushen
        socket_manager.emit_event_sync('path_complete', path_data)


# Globale Singleton-Instanz erzeugen
path_manager = PathManager()

# --- FastAPI Router Endpunkte ---

@router.post("/plan_path")
async def plan_path(req: PlanPathRequest):
    """
    REST-Endpunkt, um eine Pfadplanung zu starten.
    """
    success = path_manager.request_path(req)
    
    if success:
        return {
            "success": True, 
            "message": f"Pfadanfrage '{req.request_id}' wurde an ROS übermittelt.",
            "request_id": req.request_id
        }
    else:
        raise HTTPException(
            status_code=503, 
            detail="Der ROS Pfadplanungs-Dienst ist momentan nicht erreichbar."
        )