#!/bin/bash
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}🚀 Starte PassForm Environment...${NC}"

# 1. ROS 2 System (Jazzy)
source /opt/ros/jazzy/setup.bash

# 2. Lokaler Workspace (Nutze $HOME für absolute Sicherheit)
# WICHTIG: Ersetze 'martenwd' falls dein User anders heißt
WS_SETUP="$HOME/passform2/passform2_ws/install/setup.bash"

if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
    echo -e "${GREEN}✅ ROS Workspace geladen.${NC}"
else
    echo -e "${RED}❌ FEHLER: Workspace-Setup nicht gefunden unter: $WS_SETUP${NC}"
    echo "Bitte führe 'colcon build' im Workspace aus."
    exit 1
fi

# 3. Virtuelle Umgebung
# Wir gehen davon aus, dass .venv im passform2 Hauptordner liegt
VENV_PATH="$HOME/passform2/.venv/bin/activate"
if [ -f "$VENV_PATH" ]; then
    source "$VENV_PATH"
    echo -e "${GREEN}✅ Python Venv geladen.${NC}"
else
    echo -e "${RED}❌ FEHLER: Venv nicht gefunden unter: $VENV_PATH${NC}"
    exit 1
fi

# 4. Modul-Pfad setzen (Backend-Ordner)
export PYTHONPATH=$PYTHONPATH:$HOME/passform2/passform2_agent_backend

# 5. Server starten
echo -e "${GREEN}📡 Starte Uvicorn auf Port 8000...${NC}"
cd $HOME/passform2/passform2_agent_backend
# Geister-Prozesse auf Port 8080 finden und killen
echo "🔍 Prüfe Port 8080..."
fuser -k 8080/tcp > /dev/null 2>&1
uvicorn app.main:app --host 127.0.0.1 --port 8080 --log-level info --loop asyncio