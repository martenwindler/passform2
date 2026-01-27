# passform2

![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu%2024.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-A22846?style=for-the-badge&logo=raspberry-pi&logoColor=white)
![Eclipse BaSyx](https://img.shields.io/badge/Eclipse%20BaSyx-004A96?style=for-the-badge&logo=eclipse-ide&logoColor=white)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202%20Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=for-the-badge&logo=rust&logoColor=white)
![Elm](https://img.shields.io/badge/Elm-60B5CC?style=for-the-badge&logo=elm&logoColor=white)
![SCSS](https://img.shields.io/badge/SCSS-CC6699?style=for-the-badge&logo=sass&logoColor=white)
![Tailwind CSS](https://img.shields.io/badge/Tailwind%20CSS-06B6D4?style=for-the-badge&logo=tailwindcss&logoColor=white)


ROS2 ecosystem managing RFID events, heartbeats, system states, Raspberry Pi clusters.

## Software

```text
passform2/
├── passform2_agent_backend/  # FastAPI Orchestrator
│   ├── app/
│   │   ├── managers/         # NodeManager, AgentManager, NfcManager
│   │   ├── ros/              # ROS Client Bridge
│   │   ├── system_api.py     # REST-Schnittstelle für Orchestrierung
│   │   └── logic/            # A* Pfadplanung
│   └── data/                 # Zentrale SSoT
├── passform2_ws/             # Zentraler ROS 2 Workspace 
│   └── src/
│       ├── passform_agent_msgs/      # Passform 2 Custom Interfaces/Messages
│       ├── passform_agent_planning/  # Passform 2 ROS-Nodes (monitor_node, agent_node)
│       └── passform_ros/             # Passform 1 integration
├── deployment/               # Deployment-Vorlagen für Hardware-Module
│   └── agent_docker/         # Docker-Files RPi 5
├── frontend/                 # Web-3D-UI 
└── Makefile                  # Zentrales Build-Tool
```

Build frontend.
```text
make frontend
```

Build backend rest.
```text
make backend-rest
```

Build backend ros.
```text
make backend-ros
```

Build all.
```text
make start-all
```

## Hardware

<p align="center">
  <img src="https://github.com/user-attachments/assets/818f071a-f960-4b0f-81d4-465de21fc4ed" alt="IMG_1196" width="350">
</p>

```text
sudo apt update && sudo apt install nmap -y
```

```text
cmd.exe /c "netsh wlan show interfaces" | grep -i " SSID"
```

```text
powershell.exe -Command "netsh wlan show profile name='DEINE_SSID' key=clear" | grep -i "Inhalt"
```

```text
# on laptop
cd ~/passform2
rsync -avz --delete \
--exclude '.git' \
--exclude '__pycache__' \
--exclude 'venv/' \
--exclude '.vscode/' \
~/passform2/ {USER}@{HOST_IP}:/home/{USER}/passform2/
```

```text
# on pi
cd ~/passform2
docker compose up --build -d
sudo apt install -y spi-tools
```

```text
hostname -I | awk '{print $1}'
```

```text
netsh interface portproxy add v4tov4 listenport=8000 listenaddress=0.0.0.0 connectport=8000 connectaddress=127.0.0.1
```

```text
python3 ~/passform2/utils/pi__rfid_bridge.py
```