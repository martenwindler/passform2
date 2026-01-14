import os

def create_boilerplate():
    utils_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.abspath(os.path.join(utils_dir, ".."))
    
    ws_name = "passform2_ws"
    src_path = os.path.join(parent_dir, ws_name, "src")

    print(f"Versuche auf folgenden Pfad zuzugreifen:\n{src_path}\n")

    if not os.path.exists(src_path):
        print(f"FEHLER: Der Ordner '{src_path}' wurde nicht gefunden.")
        return

    packages = [d for d in os.listdir(src_path) if os.path.isdir(os.path.join(src_path, d))]

    for pkg in packages:
        pkg_path = os.path.join(src_path, pkg)
        print(f"Verarbeite Paket: {pkg}")

        # REKURSIVES LÖSCHEN von 'placeholder' Dateien
        for root, dirs, files in os.walk(pkg_path):
            if "placeholder" in files:
                target = os.path.join(root, "placeholder")
                try:
                    os.remove(target)
                    # Relativen Pfad für schönere Ausgabe berechnen
                    rel_path = os.path.relpath(target, pkg_path)
                    print(f"  [-] Gelöscht: {rel_path}")
                except Exception as e:
                    print(f"  [!] Fehler beim Löschen von {target}: {e}")

        # Erstelle die Standard-Struktur
        for folder in ["resource", "launch", "config", pkg]:
            os.makedirs(os.path.join(pkg_path, folder), exist_ok=True)

        # Resource Marker
        marker = os.path.join(pkg_path, "resource", pkg)
        if not os.path.exists(marker):
            with open(marker, 'w') as f: pass
            print(f"  [+] Resource Marker")

        # Python __init__.py
        init = os.path.join(pkg_path, pkg, "__init__.py")
        if not os.path.exists(init):
            with open(init, 'w') as f: pass
            print(f"  [+] Python __init__.py")

        # Launch File Template
        launch = os.path.join(pkg_path, "launch", f"{pkg}_launch.py")
        if not os.path.exists(launch):
            with open(launch, 'w') as f:
                f.write("from launch import LaunchDescription\n")
                f.write("from launch_ros.actions import Node\n\n")
                f.write("def generate_launch_description():\n")
                f.write("    return LaunchDescription([\n")
                f.write(f"        Node(package='{pkg}', executable='main', name='{pkg}_node')\n")
                f.write("    ])\n")
            print(f"  [+] Launch Template")

        # Standard Config-Datei erstellen
        config_file = os.path.join(pkg_path, "config", "params.yaml")
        if not os.path.exists(config_file):
            with open(config_file, 'w') as f:
                f.write(f"{pkg}_node:\n")
                f.write("  ros__parameters:\n")
                f.write("    update_rate: 1.0\n")
                f.write("    log_level: 'info'\n")
            print(f"  [+] Config-Template erstellt: params.yaml")

    print("\nFertig!")

if __name__ == "__main__":
    create_boilerplate()