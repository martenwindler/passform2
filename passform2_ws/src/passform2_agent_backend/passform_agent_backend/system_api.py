from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Literal

from .config import Config, SystemMode
from .managers.agent_manager import agent_manager
from .ros.ros_client import restart_ros_client
from app.socket.socket_io_manager import socket_manager

router = APIRouter()

class ModeRequest(BaseModel):
    mode: Literal["hardware", "simulation"]

@router.post("/mode")
async def switch_system_mode(request: ModeRequest):
    """Switch between hardware and simulation mode"""
    try:
        # Convert string to enum
        if request.mode == "hardware":
            new_mode = SystemMode.HARDWARE
        else:
            new_mode = SystemMode.SIMULATION
        
        # Get current mode
        current_mode = Config.get_current_mode()
        
        # Check if mode is already set
        if current_mode == new_mode:
            return {
                "message": f"Already in {request.mode} mode",
                "mode": request.mode,
                "domain_id": Config.get_domain_id()
            }
        
        # Switch mode
        Config.set_mode(new_mode)
        
        # Clear all agents when switching modes
        agent_manager.clear_all_agents()
        
        # Restart ROS client with new domain
        restart_ros_client()

        socket_manager.emit_event_sync('mode', request.mode)

        
        return {
            "message": f"Successfully switched to {request.mode} mode",
            "mode": request.mode,
            "domain_id": Config.get_domain_id()
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to switch mode: {str(e)}")

