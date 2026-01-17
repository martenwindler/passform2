# 1. sudo mkdir -p /mnt/d
# 2. sudo mount -t drvfs D: /mnt/d
import os
import getpass
from pathlib import Path

def get_ssh_public_key():
    """Sucht nach dem lokalen SSH Public Key."""
    ssh_dir = Path.home() / ".ssh"
    # Wir suchen nach modernen ed25519 Keys, dann nach klassischen rsa Keys
    key_files = ["id_ed25519.pub", "id_rsa.pub"]
    
    for key_file in key_files:
        path = ssh_dir / key_file
        if path.exists():
            print(f"[+] SSH Public Key gefunden: {key_file}")
            return path.read_text().strip()
    
    return None

def update_user_data(target_path, public_key):
    """Optimierte cloud-init Konfiguration für Ubuntu 24.04 auf RPi 5."""
    user_data_file = target_path / "user-data"
    
    # Wichtig: Wir nutzen ein f-String mit exakter Einrückung
    config_content = f"""#cloud-config
hostname: passform-pi
manage_etc_hosts: true
users:
  - name: ubuntu
    sudo: ALL=(ALL) NOPASSWD:ALL
    shell: /bin/bash
    lock_passwd: false
    ssh_authorized_keys:
      - {public_key}

# Passwort-Login aktivieren wir zur Sicherheit als Backup
ssh_pwauth: true

chpasswd:
  list: |
     ubuntu:passform2
  expire: False

# Erzwingt den Start des SSH-Dienstes, falls Cloud-Init trödelt
runcmd:
  - [ systemctl, enable, --now, ssh ]
"""
    try:
        user_data_file.write_text(config_content)
        print("[+] user-data mit SSH-Key und Fallback-Passwort (passform2) konfiguriert.")
    except Exception as e:
        print(f"[!] Fehler beim Schreiben der user-data: {e}")

def setup_raspberry_pi_boot(target_path):
    print(f"\n--- Konfiguration für {target_path.name} ---")
    
    # 1. SSH Datei für den Dienststart
    ssh_file = target_path / "ssh"
    if not ssh_file.exists():
        ssh_file.touch()
        print("[+] SSH-Dienst aktiviert.")

    # 2. Public Key auslesen und user-data schreiben
    pub_key = get_ssh_public_key()
    if pub_key:
        update_user_data(target_path, pub_key)
    else:
        print("[!] Kein SSH Public Key unter ~/.ssh/ gefunden!")
        print("    Tipp: Generiere einen mit: ssh-keygen -t ed25519")

def list_sd_contents():
    # Pfad für WSL
    wsl_path = Path("/mnt/d")
    if not wsl_path.exists():
        print(f"[-] /mnt/d nicht gefunden. Bitte 'sudo mount -t drvfs D: /mnt/d' ausführen.")
        return

    setup_raspberry_pi_boot(wsl_path)

    print(f"\n--- Aktuelle Struktur von {wsl_path} ---")
    for item in sorted(list(wsl_path.iterdir())):
        suffix = "/" if item.is_dir() else ""
        print(f"  {item.name}{suffix}")

if __name__ == "__main__":
    list_sd_contents()