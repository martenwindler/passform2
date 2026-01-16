import logging
from enum import Enum
from typing import Dict
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
    Infrastruktur-Einstellungen aus .env oder Umgebungsvariablen.
    """
    # Aktueller Betriebsmodus
    current_mode: SystemMode = SystemMode.HARDWARE
    
    # Netzwerk-Einstellungen
    backend_host: str = Field(default="0.0.0.0", alias="BACKEND_HOST")
    backend_port: int = Field(default=8000, alias="BACKEND_PORT")
    master_ip: str = Field(default="127.0.0.1", alias="MASTER_IP")
    
    # --- NEU: Heartbeat SSoT ---
    # Frequenz in Hz (Wie oft senden die Agenten?)
    heartbeat_hz: float = Field(default=1.0, alias="HEARTBEAT_HZ")
    # Sicherheitsfaktor (Wann gilt ein Agent als offline? f * factor)
    timeout_factor: float = Field(default=3.5, alias="TIMEOUT_FACTOR")
    
    # ROS 2 Domain IDs
    domain_ids: Dict[SystemMode, int] = {
        SystemMode.HARDWARE: 0,
        SystemMode.SIMULATION: 1
    }

    model_config = SettingsConfigDict(
        env_file=".env", 
        env_file_encoding='utf-8',
        extra='ignore'
    )

    @field_validator("master_ip")
    @classmethod
    def check_ip(cls, v: str) -> str:
        if v == "127.0.0.1":
            logger.warning("⚠️ MASTER_IP steht auf Localhost. Externe Clients finden das Backend so nicht!")
        return v

class ConfigManager:
    """Zentrale Instanz zur Verwaltung der Infrastruktur-Laufzeit-Einstellungen"""
    
    def __init__(self):
        try:
            self.settings = Settings()
            logger.info(f"✅ Infrastruktur-Config geladen (Modus: {self.settings.current_mode.value})")
            logger.info(f"📡 Heartbeat-SSoT: {self.settings.heartbeat_hz}Hz (Timeout-Faktor: {self.settings.timeout_factor})")
        except Exception as e:
            logger.error(f"❌ Fehler beim Laden der Infrastruktur-Config: {e}")
            self.settings = Settings(_env_file=None)

    def get_current_mode(self) -> SystemMode:
        return self.settings.current_mode

    def get_domain_id(self) -> int:
        return self.settings.domain_ids[self.settings.current_mode]

    # --- Hilfsmethoden für ROS-Parameter ---
    @property
    def heartbeat_hz(self) -> float:
        return self.settings.heartbeat_hz

    def set_heartbeat_hz(self, hz: float):
        """Aktualisiert die Heartbeat-Frequenz zur Laufzeit."""
        if hz > 0:
            self.settings.heartbeat_hz = hz
            logger.info(f"📡 SSoT Update: Heartbeat-Frequenz auf {hz}Hz gesetzt.")
            return True
        return False

    @property
    def timeout_factor(self) -> float:
        return self.settings.timeout_factor

    @property
    def heartbeat_period(self) -> float:
        """Gibt das Sendeintervall in Sekunden zurück (T = 1/f)"""
        return 1.0 / self.settings.heartbeat_hz

    def set_mode(self, mode: SystemMode) -> bool:
        if mode != self.settings.current_mode:
            old_mode = self.settings.current_mode
            self.settings.current_mode = mode
            logger.info(f"🔄 Modus-Wechsel: {old_mode.value} -> {mode.value} (ROS Domain {self.get_domain_id()})")
            return True
        return False

    @property
    def host(self) -> str:
        return self.settings.backend_host

    @property
    def port(self) -> int:
        return self.settings.backend_port

# Singleton Instanz
config = ConfigManager()
Config = config