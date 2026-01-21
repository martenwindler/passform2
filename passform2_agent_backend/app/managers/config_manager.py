import json
import os
import logging
from datetime import datetime
from typing import Dict, Any

logger = logging.getLogger("config_manager")

class ConfigManager:
    """Der ultimative SSoT Verwalter - vereint Config und Persistenz."""
    
    def __init__(self):
        # Dynamische Pfadfindung (4 Ebenen hoch zu /data/config.json)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", "data", "config.json"))
        
        self._ensure_data_dir()
        self.current_data = self.load_config()

    def _ensure_data_dir(self):
        """Stellt sicher, dass der Ordner existiert."""
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)

    def load_config(self) -> Dict[str, Any]:
        """Lädt die SSoT Datei oder erstellt Default-Werte."""
        if not os.path.exists(self.file_path):
            logger.warning("⚠️ SSoT Datei nicht gefunden, erstelle Defaults.")
            return {"config": {"grid": {"width": 10, "height": 10}}, "agents": [], "last_update": None}
        
        try:
            with open(self.file_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"❌ Fehler beim Laden: {e}")
            return {"config": {"grid": {"width": 10, "height": 10}}, "agents": []}

    def save_to_ssot(self):
        """Schreibt den aktuellen Zustand (self.current_data) auf die Platte."""
        try:
            self.current_data["last_update"] = datetime.now().isoformat()
            with open(self.file_path, 'w') as f:
                json.dump(self.current_data, f, indent=4)
            logger.info("💾 SSoT erfolgreich auf Festplatte gespeichert.")
            return True
        except Exception as e:
            logger.error(f"❌ Fehler beim Speichern: {e}")
            return False

# Singleton
config_manager = ConfigManager()