#!/usr/bin/env python3
"""
NFC Reader Script für RC522
Liest JSON-Daten von einem NFC-Chip im Format: {"x": number, "y": number}
"""

import json
import sys
import time
from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO

def read_nfc_position():
    """
    Liest Position vom NFC-Chip
    Returns: tuple (x, y) oder None bei Fehler
    """
    reader = SimpleMFRC522()
    
    print("NFC Reader bereit. Halten Sie den Chip an den Reader...")
    
    try:
        # Auf NFC-Chip warten (mit Timeout)
        start_time = time.time()
        timeout = 30  # 30 Sekunden Timeout
        
        while time.time() - start_time < timeout:
            try:
                id, text = reader.read_no_block()
                if id:
                    print(f"NFC-Chip gefunden (ID: {id})")
                    print(f"Rohdaten: {text}")
                    
                    # JSON parsen
                    try:
                        data = json.loads(text.strip())
                        x = int(data.get('x', 0))
                        y = int(data.get('y', 0))
                        
                        print(f"Position aus NFC: x={x}, y={y}")
                        return x, y
                        
                    except (json.JSONDecodeError, ValueError, KeyError) as e:
                        print(f"Fehler beim Parsen der JSON-Daten: {e}")
                        return None
                        
            except Exception as e:
                # Ignoriere Read-Fehler und versuche weiter
                pass
                
            time.sleep(0.5)
        
        print("Timeout: Kein NFC-Chip gefunden")
        return None
        
    except Exception as e:
        print(f"NFC-Reader Fehler: {e}")
        return None
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    result = read_nfc_position()
    if result:
        x, y = result
        print(f"{x},{y}")  # Ausgabe für Shell-Parsing
        sys.exit(0)
    else:
        sys.exit(1)
