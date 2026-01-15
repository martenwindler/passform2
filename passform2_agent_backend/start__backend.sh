#!/bin/bash

# Farben für die Ausgabe
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}🚀 Starte PassForm Environment...${NC}"

# 1. ROS 2 System & Workspace laden
source /opt/ros/jazzy/setup.bash
source /home/martenwd/passform2/passform2_ws/install/setup.bash

# 2. Virtuelle Umgebung laden
source .venv/bin/activate

# 3. Modul-Pfad setzen
export PYTHONPATH=$PYTHONPATH:.

# 4. Server starten
python3 -m app.main