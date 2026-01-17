import subprocess
import sys
import os

def run_script(script_name):
    print(f"\n[>>>] Starte: {script_name}")
    try:
        # Führt das Skript im gleichen Python-Interpreter aus
        result = subprocess.run([sys.executable, script_name], check=True)
        return True
    except subprocess.CalledProcessError:
        print(f"[!] Fehler in {script_name}. Abbruch.")
        return False

def main():
    print("=== Raspberry Pi 5 SD-Karten Automatisierung ===")
    
    # 1. Konfigurieren (SSH, User-Data, List)
    if not run_script("sd__write.py"):
        return

    confirm_wifi = input("\n Möchtest du WLAN-Daten hinzufügen? (j/n): ")
    if confirm_wifi.lower() == 'j':
        run_script("sd__write_router.py")

    # Kurze Bestätigung, bevor ausgeworfen wird
    confirm = input("\n Soll die Karte jetzt ausgeworfen werden? (j/n): ")
    if confirm.lower() == 'j':
        # 2. Auswerfen
        run_script("sd__eject.py")
    else:
        print("[i] Karte bleibt gemountet.")

if __name__ == "__main__":
    main()