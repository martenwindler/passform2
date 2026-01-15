import os
import logging
from enum import Enum
from typing import Dict, List, Optional
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field, field_validator

# Logger Setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("config")

class SystemMode(Enum):
    HARDWARE = "hardware"
    SIMULATION = "simulation"

class Settings(BaseSettings):
    """
    Lädt Konfigurationen aus Umgebungsvariablen oder einer .env Datei.
    Priorität: Umgebungsvariable > .env Datei > Default-Wert.
    """
    
    # --- System Status ---
    current_mode: SystemMode = SystemMode.HARDWARE
    
    # --- Netzwerk & Sicherheit (Diese Werte in die .env schreiben!) ---
    backend_host: str = Field(default="0.0.0.0", alias="BACKEND_HOST")
    backend_port: int = Field(default=8000, alias="BACKEND_PORT")
    master_ip: str = Field(default="127.0.0.1", alias="MASTER_IP")
    
    # --- ROS 2 Parameter ---
    domain_ids: Dict[SystemMode, int] = {
        SystemMode.HARDWARE: 0,
        SystemMode.SIMULATION: 1
    }
    
    # --- ROS 2 Launch Parameter ---
    agent_package: str = "passform_agent_planning"
    agent_launch_file: str = "single_agent.launch.py"

    # Konfiguration für das Einlesen der .env Datei
    model_config = SettingsConfigDict(
        env_file=".env", 
        env_file_encoding='utf-8',
        extra='ignore' # Ignoriert zusätzliche Variablen in der .env
    )

    @field_validator("master_ip")
    @classmethod
    def check_ip(cls, v: str) -> str:
        if v == "127.0.0.1":
            logger.warning("⚠️ MASTER_IP steht auf Localhost. Raspberry Pis können sich so nicht verbinden!")
        return v

class ConfigManager:
    """Zentrale Instanz zur Verwaltung der Laufzeit-Einstellungen"""
    
    def __init__(self):
        try:
            self.settings = Settings()
            logger.info(f"✅ Konfiguration geladen (Modus: {self.settings.current_mode.value})")
        except Exception as e:
            logger.error(f"❌ Fehler beim Laden der .env Konfiguration: {e}")
            # Fallback zu Standardwerten, falls .env fehlt
            self.settings = Settings(_env_file=None)

    def get_current_mode(self) -> SystemMode:
        return self.settings.current_mode

    def get_domain_id(self) -> int:
        return self.settings.domain_ids[self.settings.current_mode]

    def set_mode(self, mode: SystemMode) -> bool:
        """Wechselt den Modus und gibt True zurück bei Änderung"""
        if mode != self.settings.current_mode:
            old_mode = self.settings.current_mode
            self.settings.current_mode = mode
            logger.info(f"🔄 Modus-Wechsel: {old_mode.value} -> {mode.value} (Domain {self.get_domain_id()})")
            return True
        return False

    def get_launch_command(self, node_id: str, pos_str: str, module_type: str) -> List[str]:
        """Erstellt den ROS 2 Launch Befehl basierend auf der Config"""
        return [
            "ros2", "launch", 
            self.settings.agent_package, 
            self.settings.agent_launch_file,
            f"position:={pos_str}",
            f"module_type:={module_type}",
            f"agent_id:={node_id}"
        ]

    @property
    def host(self) -> str:
        return self.settings.backend_host

    @property
    def port(self) -> int:
        return self.settings.backend_port

# Singleton Instanz
config = ConfigManager()
Config = config # Alias für Abwärtskompatibilität