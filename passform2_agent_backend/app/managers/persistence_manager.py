import json
import os
import logging
from typing import List, Dict, Any

logger = logging.getLogger("persistence")

class PersistenceManager:
    """Verantwortlich für die dauerhafte Speicherung des Systemzustands"""
    
    def __init__(self, file_name="config.json"):
        # Dynamische Pfadfindung: Geht vom Backend-Root hoch zum Workspace-Root/data/
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", "data", file_name))
        self._ensure_data_dir()

    def _ensure_data_dir(self):
        """Stellt sicher, dass der 'data' Ordner und die Datei existieren"""
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
        if not os.path.exists(self.file_path):
            with open(self.file_path, 'w') as f:
                json.dump({"agents": [], "last_update": None}, f, indent=4)
            logger.info(f"Persistenz-Datei neu erstellt: {self.file_path}")

    def save_state(self, agents_data: List[Dict[str, Any]]):
        """Speichert den aktuellen Zustand aller Agenten"""
        try:
            from datetime import datetime
            payload = {
                "agents": agents_data,
                "last_update": datetime.now().isoformat()
            }
            with open(self.file_path, 'w') as f:
                json.dump(payload, f, indent=4)
        except Exception as e:
            logger.error(f"Fehler beim Speichern der Persistenz-Daten: {e}")

    def load_state(self) -> List[Dict[str, Any]]:
        """Lädt den letzten bekannten Zustand"""
        try:
            if os.path.exists(self.file_path):
                with open(self.file_path, 'r') as f:
                    data = json.load(f)
                    return data.get("agents", [])
        except Exception as e:
            logger.error(f"Fehler beim Laden der Persistenz-Daten: {e}")
        return []

# Singleton Instanz
persistence_manager = PersistenceManager()