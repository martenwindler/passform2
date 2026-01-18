#!/bin/bash

# Pfad-Logik
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# In das Projektverzeichnis wechseln
cd passform2_agent_frontend

echo -e "\033[1;33m>>> Arbeitsverzeichnis: $(pwd)\033[0m"

# Abhängigkeiten prüfen
if [ ! -d "node_modules" ]; then
    echo "node_modules fehlen. Installiere..."
    npm install
fi

# Bereinigung beim Beenden sicherstellen
cleanup() {
    echo -e "\n\033[1;31m>>> Beende Frontend und Tests...\033[0m"
    kill $VITE_PID
    exit
}

# Wenn das Skript abgebrochen wird, cleanup ausführen
trap cleanup SIGINT SIGTERM

echo -e "\033[1;32m>>> Starte Vite Dev-Server und Elm-Tests parallel...\033[0m"

# 1. Vite im Hintergrund starten
npm run dev &
VITE_PID=$!

# 2. Elm-Test im Vordergrund starten (Watch-Mode)
npx elm-test --watch