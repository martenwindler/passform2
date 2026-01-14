import os

def add_gitkeeps():
    utils_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.abspath(os.path.join(utils_dir, ".."))
    
    ws_name = "passform2_ws"
    ws_path = os.path.join(parent_dir, ws_name)

    print(f"Suche nach absolut leeren End-Ordnern in: {ws_path}\n")

    if not os.path.exists(ws_path):
        print(f"FEHLER: Pfad '{ws_path}' nicht gefunden.")
        return

    for root, dirs, files in os.walk(ws_path):
        # Ignoriere Systemordner
        if any(x in root for x in [".git", "build", "install", "log", "__pycache__"]):
            continue

        # Logik: Nur wenn weder Dateien NOCH Unterordner existieren
        if not dirs and not files:
            gitkeep_path = os.path.join(root, ".gitkeep")
            
            # Falls die Datei schon da ist, müssen wir nichts tun
            if not os.path.exists(gitkeep_path):
                try:
                    with open(gitkeep_path, 'w') as f:
                        pass
                    rel_path = os.path.relpath(root, ws_path)
                    print(f"  [+] .gitkeep erstellt in: {rel_path}")
                except Exception as e:
                    print(f"  [!] Fehler in {root}: {e}")

    print("\nFertig!")

if __name__ == "__main__":
    add_gitkeeps()