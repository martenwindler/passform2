import logging
from typing import Dict, List, Optional, Any
from pydantic import BaseModel
import asyncio

# Logger Setup
logger = logging.getLogger("skill_manager")

# --- DOMAIN MODELS (Ersatz für Rust Structs) ---

class Skill(BaseModel):
    id: str
    name: str
    description: str
    complexity: float

class SkillExecutionRequest(BaseModel):
    agent_id: str
    skill_id: str
    parameters: Dict[str, Any] = {}

class SkillStatusUpdate(BaseModel):
    agent_id: str
    skill_id: str
    status: str  # "running", "completed", "failed"
    progress: int  # 0-100

# --- MANAGER ---

class SkillManager:
    """
    Verwaltet die verfügbaren Fähigkeiten (Skills) und überwacht deren Ausführung.
    """
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(SkillManager, cls).__new__(cls)
            # Katalog aller verfügbaren Skills
            cls._instance.registry: Dict[str, Skill] = {}
            # Tracking laufender Skills pro Agent
            cls._instance.active_executions: Dict[str, SkillStatusUpdate] = {}
            logger.info("✅ SkillManager: Zentrale Skill-Registry bereit.")
        return cls._instance

    async def load_skills_from_ssot(self, skills_data: List[Dict[str, Any]]):
        """Lädt die Skill-Definitionen aus der SSoT (Ersatz für load_config)."""
        self.registry.clear()
        for s in skills_data:
            skill = Skill(**s)
            self.registry[skill.id] = skill
        logger.info(f"📚 {len(self.registry)} Skills in die Registry geladen.")

    async def trigger_skill(self, req: SkillExecutionRequest):
        """Triggert die Ausführung eines Skills (via ROS Bridge)."""
        
        # 1. Validierung: Existiert der Skill?
        if req.skill_id not in self.registry:
            logger.error(f"❌ Skill '{req.skill_id}' existiert nicht in der Registry!")
            return

        logger.info(f"⚡ Starte Skill '{req.skill_id}' für Agent '{req.agent_id}'")

        # 2. Initialen Status setzen
        initial_status = SkillStatusUpdate(
            agent_id=req.agent_id,
            skill_id=req.skill_id,
            status="running",
            progress=0
        )

        # 3. Status-Update verarbeiten (pusht an Frontend)
        await self.update_execution_status(initial_status)
        
        # 4. ROS Aufruf (Platzhalter)
        # from app.ros.ros_client import get_ros_client
        # get_ros_client().publish_skill_trigger(req)

    async def update_execution_status(self, update: SkillStatusUpdate):
        """Verarbeitet Status-Updates von ROS (Callback) und pusht an Elm."""
        
        logger.info(f"🔄 Skill-Update: Agent {update.agent_id} -> {update.status} ({update.progress}%)")

        # Zustand im Speicher halten
        self.active_executions[update.agent_id] = update

        # Direkt per WebSocket an das Elm-Frontend pushen
        from .socket_io_manager import socket_manager
        await socket_manager.emit_event('skill_status_update', update.dict())

        # Cleanup: Wenn fertig oder Fehler, aus aktiven Executions entfernen
        if update.status in ["completed", "failed"]:
            if update.agent_id in self.active_executions:
                del self.active_executions[update.agent_id]

# Singleton Instanz
skill_manager = SkillManager()