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


# ROS Agent Docker Container für Raspberry Pi 5

Dieses Docker-Setup ermöglicht es, den ROS Agent auf einem Raspberry Pi 5 mit NFC-Reader-Unterstützung zu betreiben.

## Hardware-Voraussetzungen

- Raspberry Pi 5
- RC522 NFC-Reader über SPI angeschlossen
- NFC-Chips mit JSON-Daten im Format: `{"x": number, "y": number}`

## RC522 Verkabelung (Raspberry Pi 5)

```
RC522    Raspberry Pi 5
VCC   -> 3.3V (Pin 1)
RST   -> GPIO25 (Pin 22)
GND   -> GND (Pin 6)
MISO  -> GPIO9/SPI0_MISO (Pin 21)
MOSI  -> GPIO10/SPI0_MOSI (Pin 19)
SCK   -> GPIO11/SPI0_SCLK (Pin 23)
SDA   -> GPIO8/SPI0_CE0 (Pin 24)
```

## Installation

1. Repository klonen und zum Verzeichnis navigieren:
```bash
git clone <your-repo-url>
cd passform_agent_ros
```

2. SPI auf dem Raspberry Pi aktivieren:
```bash
sudo raspi-config
# Interface Options -> SPI -> Enable
```

3. Docker Image bauen:
```bash
docker build -t ros-agent .
```

## Verwendung

### Option 1: Mit NFC-Reader (Standard)

Container startet, wartet auf NFC-Chip und liest Position aus:

```bash
docker run -d \
  --privileged \
  --device=/dev/gpiomem \
  --device=/dev/spidev0.0 \
  --device=/dev/spidev0.1 \
  --network=host \
  -e MODULE_TYPE=greifer \
  ros-agent
```

### Option 2: Position manuell setzen

Überspringt NFC-Reader und verwendet vorgegebene Position:

```bash
docker run -d \
  --privileged \
  --device=/dev/gpiomem \
  --network=host \
  -e MODULE_TYPE=rollen_ns \
  -e POSITION_X=3 \
  -e POSITION_Y=5 \
  ros-agent
```

### Option 3: Mit Docker Compose

```bash
# Standard-Konfiguration bearbeiten
nano docker-compose.yml

# Container starten
docker-compose up -d

# Logs anzeigen
docker-compose logs -f
```

## Umgebungsvariablen

- `MODULE_TYPE`: Art des Moduls (`greifer`, `rollen_ns`, `rollen_ow`, `mensch`)
- `POSITION_X`: X-Koordinate (überspringt NFC wenn gesetzt)
- `POSITION_Y`: Y-Koordinate (überspringt NFC wenn gesetzt)
- `SKIP_NFC`: Auf `true` setzen um NFC komplett zu überspringen

## NFC-Chip Vorbereitung

Die NFC-Chips müssen JSON-Daten im folgenden Format enthalten:

```json
{"x": 2, "y": 3}
```

Beispiel-Script zum Beschreiben eines Chips:

```python
#!/usr/bin/env python3
import json
from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO

reader = SimpleMFRC522()

try:
    data = {"x": 2, "y": 3}
    text = json.dumps(data)
    
    print("Halten Sie den Chip an den Reader...")
    reader.write(text)
    print(f"Daten geschrieben: {text}")
    
finally:
    GPIO.cleanup()
```

## Debugging

Container im interaktiven Modus starten:

```bash
docker run -it \
  --privileged \
  --device=/dev/gpiomem \
  --device=/dev/spidev0.0 \
  --device=/dev/spidev0.1 \
  --network=host \
  -e MODULE_TYPE=greifer \
  ros-agent \
  /bin/bash
```

NFC-Reader manuell testen:

```bash
python3 /nfc_reader.py
```

## Bekannte Probleme

- SPI muss auf dem Raspberry Pi aktiviert sein
- Container benötigt privileged-Modus für GPIO-Zugriff
- Bei GPIO-Fehlern: Raspberry Pi neu starten
