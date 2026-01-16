import json
import os
import logging
from datetime import datetime

logger = logging.getLogger("config_manager")

class ConfigManager:
    def __init__(self):
        # 4 Ebenen hoch: 
        # 1. managers -> 2. app -> 3. passform2_agent_backend -> 4. passform2
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        self.file_path = os.path.join(base_dir, "data", "config.json")
        
        logger.info(f"🔍 SSoT Pfad: {self.file_path}")
        self.current_data = self.load_config()

    def load_config(self):
        if not os.path.exists(self.file_path):
            logger.error(f"❌ SSoT Datei nicht gefunden!")
            return {"config": {"grid": {"width": 10, "height": 10}}, "agents": []}
        
        try:
            with open(self.file_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"❌ Fehler beim Lesen der Config: {e}")
            return {"config": {"grid": {"width": 10, "height": 10}}, "agents": []}

    def save_to_ssot(self):
        try:
            with open(self.file_path, 'w') as f:
                json.dump(self.current_data, f, indent=4)
            return True
        except Exception as e:
            logger.error(f"❌ Fehler beim Speichern: {e}")
            return False

config_manager = ConfigManager()