import time
import requests
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

# Adresse deines Backends (Master)
BACKEND_URL = "http://192.168.178.50:8000/api/system/rfid_scan"

reader = SimpleMFRC522()

print("📟 PassForm 2.0 RFID-Reader bereit...")

try:
    while True:
        print("\nWarte auf Tag...")
        id, text = reader.read()
        print(f"✅ Chip erkannt: {id}")
        
        payload = {"rfid_id": str(id)}
        
        try:
            response = requests.post(BACKEND_URL, json=payload, timeout=2)
            if response.status_code == 200:
                print("📤 Daten erfolgreich an Backend gesendet.")
            else:
                print(f"❌ Backend Fehler: {response.status_code}")
        except Exception as e:
            print(f"❌ Verbindung zum Backend fehlgeschlagen: {e}")
        
        # Kurze Pause, um Doppelscans zu vermeiden
        time.sleep(2)

finally:
    GPIO.cleanup()