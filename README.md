# passform2

![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu%2024.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-A22846?style=for-the-badge&logo=raspberry-pi&logoColor=white)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202%20Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Eclipse BaSyx](https://img.shields.io/badge/Eclipse%20BaSyx-004A96?style=for-the-badge&logo=eclipse-ide&logoColor=white)
![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=for-the-badge&logo=rust&logoColor=white)
![Tokio](https://img.shields.io/badge/Tokio-000000?style=for-the-badge&logo=rust&logoColor=white) 
![Elm](https://img.shields.io/badge/Elm-60B5CC?style=for-the-badge&logo=elm&logoColor=white)
![Tailwind CSS](https://img.shields.io/badge/Tailwind%20CSS-06B6D4?style=for-the-badge&logo=tailwindcss&logoColor=white)
![SCSS](https://img.shields.io/badge/SCSS-CC6699?style=for-the-badge&logo=sass&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-2496ED?style=for-the-badge&logo=docker&logoColor=white)
![Vite](https://img.shields.io/badge/Vite-646CFF?style=for-the-badge&logo=vite&logoColor=white)

ROS2 ecosystem managing RFID events, heartbeats, system states, Raspberry Pi clusters.

## Software

```text
passform2/
├── passform2_agent_backend/    # Zentraler Orchestrator (Hybrid)
│   ├── src/                    # Modern Rust Core (Axum, rclrs, Tokio)
│   │   ├── managers/           # Asynchrone Logik (Agent-, Path-, Skill-Manager)
│   │   ├── ros/                # ROS 2 Jazzy Bridge (rclrs Client)
│   │   └── main.rs             # API-Entrypoint & Task-Orchestrierung
│   ├── app/                    # Legacy Python Backend (FastAPI)
│   ├── start__backend.sh       # Launcher (Standard: Rust, --legacy: Python)
│   └── Cargo.toml              # Rust Dependencies & Build-Config
├── passform2_agent_frontend/   # Web-3D-UI
│   ├── src/                    # Frontend Logik
│   ├── index.html              # 3D-Szenen Entrypoint
│   └── vite.config.js          # Vite Build-System & Dev-Server
├── passform2_ws/               # Zentraler ROS 2 Workspace (Jazzy)
│   └── src/
│       ├── passform_agent_resources/ # Custom Messages & Services (SSoT)
│       └── ros2_rust/          # rclrs & Code-Generatoren Source-Build
├── data/                       # Zentrale SSoT (Konfigurationen & JSON-Data)
├── deployment/                 # Docker- & Hardware-Provisionierung (RPi 5)
├── utils/                      # Administrative Shell-Scripts & Maintenance-Tools
└── Makefile                    # Globales Steuerungs-Tool (make install, make start-all)
```

Quickstart.
```text
make install
```

Build build-ws.
```text
make build-ws
```

Build backend.
```text
make backend
```

(Build backend-legacy python).
```text
make backend-legacy
```

Inspect backend. 
```text
http://127.0.0.1:8080/
```

Build agent.
```text
ros2 topic pub -1 /agent_info passform_agent_resources/msg/AgentInfo "{agent_id: 'Robby-01', module_type: 'Y-Module', position: {x: 5, y: 2}, orientation: 90.0}"
```

Inspect ROS system state.
```text
http://127.0.0.1:8080/
```

Inspect agent list.
```text
http://127.0.0.1:8080/api/agents
```

Inspect basyx.
```text
http://127.0.0.1:8080/basyx/
```

Build frontend.
```text
make frontend
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