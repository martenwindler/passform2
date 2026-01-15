#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

cd passform2_agent_frontend

echo -e "\033[1;33m>>> Arbeitsverzeichnis: $(pwd)\033[0m"

if [ ! -d "node_modules" ]; then
    echo "node_modules fehlen. Installiere..."
    npm install
fi

npm run dev