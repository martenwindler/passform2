import os
from pathlib import Path

def add_wifi_config(target_path, ssid, password):
    user_data_file = target_path / "user-data"
    
    if not user_data_file.exists():
        print("[!] user-data Datei nicht gefunden. Bitte zuerst sd__manage.py ausführen.")
        return

    # Die Netzwerk-Konfiguration für Netplan
    wifi_snippet = f"""
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "{ssid}":
          password: "{password}"
"""

    try:
        # Wir hängen die Netzwerk-Konfiguration einfach ans Ende der Datei
        with open(user_data_file, "a") as f:
            f.write(wifi_snippet)
        print(f"[+] WLAN-Konfiguration für '{ssid}' hinzugefügt.")
    except Exception as e:
        print(f"[!] Fehler beim Schreiben der WLAN-Daten: {e}")

def main():
    target_path = Path("/mnt/d")
    if not target_path.exists():
        print("[-] Laufwerk /mnt/d nicht gefunden.")
        return

    print("--- WLAN / Router Konfiguration ---")
    ssid = input("SSID (WLAN-Name): ")
    password = input("WLAN-Passwort: ")

    if ssid and password:
        add_wifi_config(target_path, ssid, password)
    else:
        print("[!] SSID oder Passwort leer. Abbruch.")

if __name__ == "__main__":
    main()