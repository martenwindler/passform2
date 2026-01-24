#!/bin/bash

# Generisches Skript zum Erstellen von .gitkeep Dateien in leeren Ordnern
TARGET_DIR="$1"

# Hilfe-Anzeige
if [ -z "$TARGET_DIR" ]; then
    echo "Usage: $0 <path_to_subtree>"
    echo "Example: $0 passform2_agent_backend/src"
    exit 1
fi

# Prüfen, ob das Verzeichnis existiert
if [ ! -d "$TARGET_DIR" ]; then
    echo "❌ Fehler: Verzeichnis '$TARGET_DIR' existiert nicht."
    exit 1
fi

echo "🔍 Suche nach leeren Ordnern in: $TARGET_DIR"

# Logik:
# 1. -type d: Suche nur Verzeichnisse
# 2. -empty: Nur wenn sie wirklich leer sind
# 3. -not -path '*/.*': Ignoriere versteckte Ordner (wie .git)
# 4. -exec touch ...: Erstelle die Datei
find "$TARGET_DIR" -type d -empty -not -path '*/.*' -exec touch {}/.gitkeep \; -print | while read -r line; do
    echo "  ✅ .gitkeep erstellt in: $line"
done

echo "✨ Fertig!"