# config.py
import os
from enum import Enum

class SystemMode(Enum):
    HARDWARE = "hardware"
    SIMULATION = "simulation"

class Config:
    _instance = None
    _current_mode = SystemMode.HARDWARE
    _domain_ids = {
        SystemMode.HARDWARE: 0,
        SystemMode.SIMULATION: 1
    }
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    @classmethod
    def get_current_mode(cls) -> SystemMode:
        return cls._current_mode
    
    @classmethod
    def get_domain_id(cls) -> int:
        return cls._domain_ids[cls._current_mode]
    
    @classmethod
    def set_mode(cls, mode: SystemMode) -> bool:
        """Wechselt den Modus und gibt True zurück wenn sich etwas geändert hat"""
        if mode != cls._current_mode:
            old_mode = cls._current_mode
            cls._current_mode = mode
            print(f"Mode gewechselt: {old_mode.value} -> {mode.value} (Domain {cls.get_domain_id()})")
            return True
        return False

# Globale Instanz für Backward-Kompatibilität
config = Config()
