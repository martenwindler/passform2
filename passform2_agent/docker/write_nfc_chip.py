#!/usr/bin/env python3
"""
Hilfsskript zum Beschreiben von NFC-Chips mit Position-Daten
"""

import json
import sys
from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO

def write_position_to_nfc(x, y):
    """
    Schreibt Position auf NFC-Chip
    """
    reader = SimpleMFRC522()
    
    try:
        data = {"x": x, "y": y}
        text = json.dumps(data)
        
        print(f"Schreibe Position x={x}, y={y} auf NFC-Chip...")
        print(f"JSON-Daten: {text}")
        print("Halten Sie den Chip an den Reader...")
        
        reader.write(text)
        print("✓ Daten erfolgreich geschrieben!")
        
        # Verifikation
        print("Lese zur Verifikation...")
        id, read_text = reader.read()
        print(f"Gelesene Daten: {read_text}")
        
        return True
        
    except Exception as e:
        print(f"Fehler beim Schreiben: {e}")
        return False
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Verwendung: python3 write_nfc_chip.py <x> <y>")
        print("Beispiel: python3 write_nfc_chip.py 2 3")
        sys.exit(1)
    
    try:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        
        if write_position_to_nfc(x, y):
            sys.exit(0)
        else:
            sys.exit(1)
            
    except ValueError:
        print("Fehler: x und y müssen ganze Zahlen sein")
        sys.exit(1)
