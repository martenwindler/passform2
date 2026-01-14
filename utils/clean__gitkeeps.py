import os

def remove_gitkeeps():
    utils_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.abspath(os.path.join(utils_dir, ".."))
    
    ws_name = "passform2_ws"
    ws_path = os.path.join(parent_dir, ws_name)

    print(f"Suche nach .gitkeep Dateien zum Löschen in: {ws_path}\n")

    if not os.path.exists(ws_path):
        print(f"FEHLER: Pfad '{ws_path}' nicht gefunden.")
        return

    count = 0
    for root, dirs, files in os.walk(ws_path):
        # Wir löschen überall, außer im .git Ordner selbst (Sicherheitsmaßnahme)
        if ".git" in root:
            continue

        if ".gitkeep" in files:
            file_path = os.path.join(root, ".gitkeep")
            try:
                os.remove(file_path)
                rel_path = os.path.relpath(file_path, ws_path)
                print(f"  [-] Gelöscht: {rel_path}")
                count += 1
            except Exception as e:
                print(f"  [!] Fehler beim Löschen von {file_path}: {e}")

    print(f"\nFertig! Insgesamt {count} .gitkeep Datei(en) entfernt.")

if __name__ == "__main__":
    remove_gitkeeps()