## Commands

### Monitor launch

```bash
ros2 launch passform_agent_planning monitor.launch.py
```

### Example Grid

```bash
ros2 launch passform_agent_planning dynamic_grid.launch.py
```

### Path Request Message

```bash
ros2 topic pub /path_request passform_agent_msgs/msg/PathRequest "{request_id: 'req0', start: {x: 0, y: 0}, goal: {x: 1, y: 1}}" -1
```

# ROS Package mit Docker Integration

Dieses Repository enthält das `passform_agent_ros` ROS-Package mit integrierter Docker-Unterstützung für Raspberry Pi 5.

## Repository-Struktur

```
passform_agent_ros/
├── passform_agent_msgs/          # ROS Message Definitionen
├── passform_agent_planning/      # Hauptpaket mit Agent-Logik
│   ├── scripts/
│   │   └── agent_node.py        # Haupt-Agent-Node
│   └── launch/
│       └── single_agent.launch.py
├── docker/                      # Docker-Hilfsskripte
│   ├── entrypoint.sh           # Container-Startup-Skript
│   ├── nfc_reader.py           # NFC-Reader für RC522
│   └── write_nfc_chip.py       # NFC-Chip beschreiben
├── Dockerfile                   # Multi-Arch Docker Image
├── docker-compose.yml          # Container-Orchestrierung
├── .dockerignore              # Docker Build-Ausschlüsse
└── README_Docker.md           # Detaillierte Docker-Dokumentation
```

## Schnellstart

### Lokale ROS-Entwicklung
```bash
# Workspace bauen
colcon build
source install/setup.bash

# Single Agent starten
ros2 launch passform_agent_planning single_agent.launch.py module_type:=greifer position:="[1,2]"
```

### Docker Deployment (Raspberry Pi 5)
```bash
# Repository klonen
git clone <repo-url>
cd passform_agent_ros

# Mit NFC-Reader (automatische Positionserkennung)
docker-compose up -d

# Oder mit manueller Position
docker run -d --privileged --network=host \
  -e MODULE_TYPE=rollen_ns -e POSITION_X=3 -e POSITION_Y=5 \
  $(docker build -q .)
```

Siehe [README_Docker.md](README_Docker.md) für detaillierte Docker-Anweisungen.

---
