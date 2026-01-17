import subprocess
import os
import sys

def eject_sd():
    drive_letter = "D"
    mount_path = f"/mnt/{drive_letter.lower()}"

    print(f"[*] Vorbereitung zum Auswerfen von Laufwerk {drive_letter}:...")

    # 1. Prüfen, ob wir selbst im Verzeichnis sind
    if os.getcwd().startswith(mount_path):
        print(f"[!] Fehler: Dein Terminal befindet sich noch in {mount_path}!")
        print("    Bitte wechsle mit 'cd ~' in dein Home-Verzeichnis.")
        return

    # 2. In WSL unmounten
    print(f"[*] Unmounte {mount_path} in WSL...")
    try:
        # 'sudo' ist hier meist nötig
        subprocess.run(["sudo", "umount", mount_path], check=True)
        print("[+] WSL-Unmount erfolgreich.")
    except subprocess.CalledProcessError:
        print(f"[!] Fehler beim Unmounten. Ist eine Datei noch geöffnet?")
        return

    # 3. Windows-Befehl zum Auswerfen via PowerShell senden
    print(f"[*] Sende Auswurf-Befehl an Windows für Laufwerk {drive_letter}:...")
    
    # Dieser PowerShell-Befehl spricht das Windows-Shell-Objekt an, um 'Eject' zu drücken
    ps_cmd = (
        f"$drive = '{drive_letter}:'; "
        "(New-Object -ComObject Shell.Application).Namespace(17).ParseName($drive).InvokeVerb('Eject')"
    )
    
    try:
        subprocess.run(["powershell.exe", "-Command", ps_cmd], check=True)
        print(f"[+++] Erfolg! Die SD-Karte kann jetzt sicher entfernt werden.")
    except Exception as e:
        print(f"[!] Fehler beim Windows-Auswurf: {e}")

if __name__ == "__main__":
    eject_sd()