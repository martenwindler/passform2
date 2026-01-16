import threading
import time
import logging
import json

logger = logging.getLogger("nfc_manager")

# Versuche Hardware-Bibliotheken zu laden
try:
    from mfrc522 import SimpleMFRC522
    import RPi.GPIO as GPIO
    HAS_HARDWARE = True
except (ImportError, RuntimeError):
    HAS_HARDWARE = False
    logger.warning("⚠️ NFC-Hardware nicht erkannt oder kein Raspberry Pi. Dummy-Modus aktiv.")

class NfcManager:
    def __init__(self):
        self.reader = None
        self.hardware_status = "missing"
        self.running = False
        self.thread = None
        
        if HAS_HARDWARE:
            try:
                # Kurzes Setup, um GPIO-Warnungen zu vermeiden
                self.reader = SimpleMFRC522()
                self.hardware_status = "online"
                logger.info("✅ RFID-RC522 erfolgreich initialisiert.")
            except Exception as e:
                logger.error(f"❌ Fehler bei SimpleMFRC522 Initialisierung: {e}")
                self.hardware_status = "error"
        else:
            self.hardware_status = "missing"

    def get_status(self):
        """Gibt den aktuellen Status zurück: 'online', 'missing' oder 'error'."""
        return self.hardware_status

    # --- SCHREIB-LOGIK ---

    def write_position(self, x: int, y: int, timeout: int = 10):
        """Wandelt x und y in JSON um und schreibt es auf den Chip."""
        data_json = json.dumps({"x": x, "y": y})
        return self.write_tag(data_json, timeout, verify=True)

    def write_tag(self, data: str, timeout: int = 10, verify: bool = False):
        """Allgemeine Schreibmethode für Strings mit optionaler Verifikation."""
        if self.hardware_status != "online" or not self.reader:
            return "no_hardware"

        logger.info(f"💾 Warte auf Chip zum Schreiben von: {data}")
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            try:
                # .write() blockiert kurzzeitig
                self.reader.write(data)
                
                if verify:
                    time.sleep(0.3)
                    id, read_text = self.reader.read_no_block()
                    if read_text and read_text.strip() == data.strip():
                        logger.info("✅ Schreiben erfolgreich verifiziert.")
                        return "success"
                    else:
                        continue
                
                logger.info("✅ Schreiben erfolgreich.")
                return "success"
                
            except Exception:
                # Meistens "No tag found" Fehler
                pass
            
            time.sleep(0.5)
            
        return "timeout"

    # --- LESE-LOGIK ---

    def read_position(self, timeout: int = 20):
        """Liest Koordinaten vom Chip (für Startup/Initialisierung)."""
        if self.hardware_status != "online": 
            return None
            
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                id, text = self.reader.read_no_block()
                if id and text:
                    data = json.loads(text.strip())
                    return int(data.get('x', 0)), int(data.get('y', 0))
            except: 
                pass
            time.sleep(0.5)
        return None

    def start_reading(self):
        """Startet den Hintergrund-Thread für RFID-Scans."""
        if self.hardware_status == "online" and self.reader and not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            logger.info("📡 NFC Hintergrund-Thread gestartet.")

    def _read_loop(self):
        """Kontinuierliche Schleife für den RFID-Scanner."""
        while self.running:
            try:
                # read() blockiert, bis ein Tag kommt
                id, text = self.reader.read()
                
                # Import hier, um circular imports zu vermeiden
                from app.socket.socket_io_manager import socket_manager
                socket_manager.emit_event_sync('rfid_found', str(id).strip())
                
                time.sleep(2) # Kurze Pause nach Scan
            except Exception as e:
                logger.error(f"RFID Loop Error: {e}")
                time.sleep(5)

# Singleton Instanz
nfc_manager = NfcManager()