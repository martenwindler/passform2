import socketio
import time
from mfrc522 import SimpleMFRC522

# Socket.io Client initialisieren
sio = socketio.Client()
reader = SimpleMFRC522()

PI_ID = "PassForM2-Pi5-Client"
SERVER_URL = "http://192.168.178.78:8000"

@sio.event
def connect():
    print(f"✅ Verbunden mit Agent-Backend auf {SERVER_URL}")
    # Hardware beim Server registrieren
    sio.emit('pi_hardware_update', {
        "pi_id": PI_ID,
        "rfid_status": "online",
        "pi_exists": True
    })

def run_bridge():
    try:
        sio.connect(SERVER_URL)
        print("📡 RFID-Scan aktiv... (Warte auf Karte)")
        while True:
            id, text = reader.read_no_block() 
            if id:
                print(f"🪪 KARTE ERKANNT! ID: {id}")
                # Sende die ID und welche Pi sie gescannt hat ans Backend
                sio.emit('rfid_scanned', {"id": id, "pi_id": PI_ID})
                # Kurze Pause nach erfolgreichem Scan, um Mehrfach-Reads zu vermeiden
                time.sleep(1)
            
            time.sleep(0.1) # Schnellerer Loop für bessere Reaktionszeit
    except Exception as e:
        print(f"❌ Fehler: {e}")
    finally:
        if sio.connected:
            sio.disconnect()

if __name__ == "__main__":
    time.sleep(5)
    run_bridge()