import os

def create_boilerplate():
    utils_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.abspath(os.path.join(utils_dir, ".."))
    
    ws_name = "passform2_ws"
    src_path = os.path.join(parent_dir, ws_name, "src")

    print(f"Suche nach ROS-Paketen in: {src_path}\n")

    if not os.path.exists(src_path):
        print(f"FEHLER: Der Ordner '{src_path}' wurde nicht gefunden.")
        return

    # Wir suchen rekursiv nach Ordnern, die eine package.xml enthalten
    for root, dirs, files in os.walk(src_path):
        if "package.xml" in files:
            pkg_path = root
            pkg = os.path.basename(root)
            print(f"\nVerarbeite Paket: {pkg} ({os.path.relpath(pkg_path, src_path)})")

            # 1. REKURSIVES LÖSCHEN von 'placeholder' Dateien im Paket
            for p_root, p_dirs, p_files in os.walk(pkg_path):
                if "placeholder" in p_files:
                    target = os.path.join(p_root, "placeholder")
                    try:
                        os.remove(target)
                        print(f"  [-] Gelöscht: {os.path.relpath(target, pkg_path)}")
                    except Exception as e:
                        print(f"  [!] Fehler beim Löschen von {target}: {e}")

            # 2. Standard-Struktur erstellen
            for folder in ["resource", "launch", "config", pkg]:
                os.makedirs(os.path.join(pkg_path, folder), exist_ok=True)

            # 3. Resource Marker
            marker = os.path.join(pkg_path, "resource", pkg)
            if not os.path.exists(marker):
                with open(marker, 'w') as f: pass
                print(f"  [+] Resource Marker")

            # 4. Python __init__.py
            init = os.path.join(pkg_path, pkg, "__init__.py")
            if not os.path.exists(init):
                with open(init, 'w') as f: pass
                print(f"  [+] Python __init__.py")

            # 5. Launch File Template
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

            # 6. Standard Config-Datei erstellen
            config_file = os.path.join(pkg_path, "config", "params.yaml")
            if not os.path.exists(config_file):
                with open(config_file, 'w') as f:
                    f.write(f"{pkg}_node:\n")
                    f.write("  ros__parameters:\n")
                    f.write("    update_rate: 1.0\n")
                    f.write("    log_level: 'info'\n")
                print(f"  [+] Config-Template")

            # 7. setup.cfg erstellen
            setup_cfg = os.path.join(pkg_path, "setup.cfg")
            if not os.path.exists(setup_cfg):
                with open(setup_cfg, 'w') as f:
                    f.write("[develop]\n")
                    f.write(f"script_dir=$base/lib/{pkg}\n")
                    f.write("[install]\n")
                    f.write(f"install_scripts=$base/lib/{pkg}\n")
                print(f"  [+] setup.cfg erstellt")

    print("\nAlle Pakete wurden aktualisiert!")

if __name__ == "__main__":
    create_boilerplate()