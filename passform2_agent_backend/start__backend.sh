#!/bin/bash

# Farben für bessere Lesbarkeit
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}🚀 PassForm 2 Core - Multi-Backend & UI Launcher ${NC}"
echo -e "${BLUE}================================================${NC}"

# 1. Grund-Setup: ROS 2 Jazzy laden
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✅ ROS 2 Jazzy System-Setup geladen.${NC}"
else
    echo -e "${RED}❌ FEHLER: ROS 2 Jazzy nicht gefunden in /opt/ros/jazzy/${NC}"
    exit 1
fi

# 2. Workspace-Setup
WS_SETUP="$HOME/passform2/passform2_ws/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
    echo -e "${GREEN}✅ Lokaler Workspace (passform2_ws) geladen.${NC}"
else
    echo -e "${YELLOW}⚠️  WARNUNG: Workspace-Setup nicht gefunden. Versuche colcon build...${NC}"
    cd "$HOME/passform2/passform2_ws" && colcon build --packages-up-to rclrs passform_agent_resources
    source "$WS_SETUP"
fi

# 3. BaSyx UI Check (Statischer Content für das Dashboard)
BACKEND_DIR="$HOME/passform2/passform2_agent_backend"
UI_PATH="$BACKEND_DIR/static/basyx-ui"

if [ ! -d "$UI_PATH" ]; then
    echo -e "${YELLOW}📥 BaSyx UI fehlt. Initialisiere Verzeichnisse...${NC}"
    mkdir -p "$BACKEND_DIR/static"
    mkdir -p "$BACKEND_DIR/repos"
    
    # Falls das Repo noch nicht da ist, klonen wir nur das Nötigste (Sparse Checkout)
    if [ ! -d "$BACKEND_DIR/repos/basyx-applications" ]; then
        echo -e "${YELLOW}🔍 Klone BaSyx Web UI Ressourcen...${NC}"
        git clone --depth 1 https://github.com/eclipse-basyx/basyx-applications.git "$BACKEND_DIR/repos/basyx-applications"
        cp -r "$BACKEND_DIR/repos/basyx-applications/basyx-ui" "$UI_PATH"
        echo -e "${GREEN}✅ BaSyx UI Assets kopiert.${NC}"
    fi
else
    echo -e "${GREEN}✅ BaSyx UI Assets einsatzbereit.${NC}"
fi

# 4. Port 8080 freimachen
echo -e "${YELLOW}🔍 Bereinige Port 8080...${NC}"
fuser -k 8080/tcp > /dev/null 2>&1

# 5. Verzweigungs-Logik
cd "$BACKEND_DIR"

if [[ "$1" == "--legacy" || "$1" == "-l" ]]; then
    # --- MODUS: LEGACY (Python) ---
    echo -e "${YELLOW}📜 Starte LEGACY-BACKEND (Python /app/)...${NC}"
    
    VENV_PATH="$HOME/passform2/.venv/bin/activate"
    if [ -f "$VENV_PATH" ]; then
        source "$VENV_PATH"
        echo -e "${GREEN}✅ Python Venv aktiviert.${NC}"
    else
        echo -e "${RED}❌ FEHLER: Virtuelle Umgebung nicht gefunden.${NC}"
        exit 1
    fi

    export PYTHONPATH=$PYTHONPATH:$(pwd)
    # BaSyx UI wird im Python-Modus meist über einen separaten statischen Mount in FastAPI serviert
    uvicorn app.main:app --host 127.0.0.1 --port 8080 --log-level info

else
    # --- MODUS: MODERN (Rust - Standard) ---
    echo -e "${GREEN}🦀 Starte MODERN-BACKEND (Rust src/)...${NC}"
    
    # Linker-Pfad für ROS 2 Jazzy Bibliotheken
    export RUSTFLAGS="-C link-arg=-Wl,-rpath,/opt/ros/jazzy/lib"
    
    # BaSyx UI wird hier von Axum (Rust) als Static Dir serviert
    echo -e "${BLUE}🌐 Dashboard verfügbar unter http://localhost:8080/basyx/ ${NC}"
    cargo run
fi