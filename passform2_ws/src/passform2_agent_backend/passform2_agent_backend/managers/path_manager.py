# ros_agent_router.py

import signal
import subprocess
import threading
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Dict, Tuple


router = APIRouter()

class Position(BaseModel):
    x: int
    y: int

class PlanPathRequest(BaseModel):
    request_id: str
    start: Position
    goal: Position
    central: bool = False  # Ob die Anfrage an den zentralen Agenten gesendet werden soll



class PathManager:
    def __init__(self):
        self.path_requests: Dict[str, PlanPathRequest] = {}  
        self.path_responses: Dict[str, Dict] = {}  

    def add_request(self, req: PlanPathRequest):
        """Fügt eine Path-Request hinzu und versucht sie zu senden"""
        self.path_requests[req.request_id] = req
        
        try:
            from app.ros.ros_client import get_ros_client
            ros_client = get_ros_client()
            ros_client.send_path_request(req, useCentral=req.central)
            print(f"✅ PathRequest {req.request_id} erfolgreich gesendet")
            return True
                
        except Exception as e:
            print(f"❌ Fehler beim Senden der PathRequest {req.request_id}: {e}")
            # Request aus der Liste entfernen, da fehlgeschlagen
            if req.request_id in self.path_requests:
                del self.path_requests[req.request_id]
            return False
    
    def add_response(self, path: Dict):
        print(path)
        req_id = path.get('request_id')
        if req_id in self.path_requests:
            self.path_responses[req_id] = path
            del self.path_requests[req_id]
        
            from app.socket.socket_io_manager import socket_manager
            socket_manager.emit_event_sync('path_complete', path)
        else:
            raise ValueError(f"Request ID '{req_id}' not found in path requests.")




path_manager = PathManager()


@router.post("/plan_path")
async def plan_path(req: PlanPathRequest):
    try:
        print(f"📍 Plane Pfad von ({req.start.x}, {req.start.y}) zu ({req.goal.x}, {req.goal.y})")
        
        success = path_manager.add_request(req)
        
        if success:
            return {
                "success": True, 
                "message": f"PathRequest erfolgreich gesendet für: '{req.request_id}'",
                "request_id": req.request_id
            }
        else:
            raise HTTPException(
                status_code=503, 
                detail=f"PathRequest konnte nicht gesendet werden. ROS-System möglicherweise nicht verfügbar."
            )
            
    except HTTPException:
        # HTTPException direkt weiterleiten
        raise
    except Exception as e:
        print(f"❌ Fehler im plan_path Endpunkt: {e}")
        raise HTTPException(status_code=500, detail=str(e))

