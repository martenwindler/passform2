import threading
import time
import logging
import json

logger = logging.getLogger("nfc_manager")

try:
    from mfrc522 import SimpleMFRC522
    import RPi.GPIO as GPIO
    HAS_HARDWARE = True
except (ImportError, RuntimeError):
    HAS_HARDWARE = False
    logger.warning("⚠️ NFC-Hardware nicht erkannt. Dummy-Modus aktiv.")

class NfcManager:
    def __init__(self):
        self.reader = None
        self.hardware_status = "missing"
        self.running = False
        self.thread = None
        
        if HAS_HARDWARE:
            try:
                self.reader = SimpleMFRC522()
                self.hardware_status = "online"
                logger.info("✅ RFID-RC522 initialisiert.")
            except Exception as e:
                logger.error(f"❌ Initialisierungsfehler: {e}")
                self.hardware_status = "error"

    def get_status(self):
        return self.hardware_status

    # --- SCHREIB-LOGIK (Ersatz für write_nfc_chip.py) ---

    def write_position(self, x: int, y: int, timeout: int = 10):
        """
        Wandelt x und y in JSON um und schreibt es auf den Chip.
        Inklusive Verifikation (wie im alten Skript).
        """
        data_json = json.dumps({"x": x, "y": y})
        return self.write_tag(data_json, timeout, verify=True)

    def write_tag(self, data: str, timeout: int = 10, verify: bool = False):
        """
        Allgemeine Schreibmethode für beliebige Strings.
        """
        if self.hardware_status != "online" or not self.reader:
            return "no_hardware"

        logger.info(f"💾 Warte auf Chip zum Schreiben von: {data}")
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            try:
                # 1. Versuche zu schreiben
                self.reader.write(data)
                
                # 2. Optionale Verifikation (wie in deinem Hilfsskript)
                if verify:
                    time.sleep(0.2)
                    id, read_text = self.reader.read_no_block()
                    if read_text and read_text.strip() == data.strip():
                        logger.info("✅ Schreiben erfolgreich verifiziert.")
                        return "success"
                    else:
                        logger.warning("⚠️ Verifikation fehlgeschlagen, versuche erneut...")
                        continue
                
                logger.info("✅ Schreiben erfolgreich.")
                return "success"
                
            except Exception as e:
                # Falls kein Chip da ist, wirft .write() oft eine Exception
                pass
            
            time.sleep(0.5)
            
        return "timeout"

    # --- LESE-LOGIK ---

    def read_position(self, timeout: int = 20):
        if self.hardware_status != "online": return None
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                id, text = self.reader.read_no_block()
                if id and text:
                    data = json.loads(text.strip())
                    return int(data.get('x', 0)), int(data.get('y', 0))
            except: pass
            time.sleep(0.5)
        return None

    def start_reading(self):
        if self.hardware_status == "online" and self.reader and not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()

    def _read_loop(self):
        while self.running:
            try:
                id, text = self.reader.read()
                from app.socket.socket_io_manager import socket_manager
                socket_manager.emit_event_sync('rfid_found', str(id).strip())
                time.sleep(2)
            except: time.sleep(5)

nfc_manager = NfcManager()