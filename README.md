# passform2

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
│       ├── passform_agent_msgs/      # Custom Interfaces/Messages
│       └── passform_agent_planning/  # ROS-Nodes (monitor_node, agent_node)
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

