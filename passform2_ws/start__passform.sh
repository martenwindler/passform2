#!/bin/bash

# Pfade definieren
WS_PATH=~/passform2/passform2_ws
FRONTEND_PATH=~/passform2
VENV_PATH="$FRONTEND_PATH/.venv"

echo "🚀 Starte Passform 2 ROS-System (Jazzy Environment)..."

# 1. Virtual Environment aktivieren
if [ -f "$VENV_PATH/bin/activate" ]; then
    echo "🐍 Aktiviere Virtual Environment..."
    # Wir nutzen den Punkt-Operator für maximale Kompatibilität
    . "$VENV_PATH/bin/activate"
else
    echo "❌ FEHLER: Keine .venv in $FRONTEND_PATH gefunden!"
    echo "Bitte erstelle sie mit: python3 -m venv .venv --system-site-packages"
    exit 1
fi

# 2. ROS-Umgebung (Jazzy) finden
ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
    # Fallback falls Jazzy nicht an Standardort
    ROS_SETUP=$(ls /opt/ros/*/setup.bash 2>/dev/null | head -n 1)
fi

if [ -z "$ROS_SETUP" ]; then
    echo "❌ FEHLER: Keine ROS 2 Installation unter /opt/ros/ gefunden!"
    exit 1
else
    echo "📦 Nutze ROS Setup: $ROS_SETUP"
    . "$ROS_SETUP"
fi

# 3. Workspace Build
cd "$WS_PATH" || exit

# CMake-Fix: Sicherstellen, dass Verzeichnisse existieren
mkdir -p src/passform_agent_planning/config

# Build ausführen
colcon build --packages-select passform_agent_planning passform_agent_msgs
if [ $? -ne 0 ]; then
    echo "❌ Build fehlgeschlagen."
    exit 1
fi

# Lokalen Workspace sourcen
. install/setup.bash

# Cleanup-Funktion (beim Beenden mit Strg+C)
cleanup() {
    echo -e "\n🛑 Fahre ROS-Backend herunter..."
    # Killt die Hintergrundprozesse sauber
    kill $BRIDGE_PID $MONITOR_PID $TEST_AGENT_PID 2>/dev/null
    exit
}
trap cleanup INT

# 4. Prozesse starten
# Da die venv aktiviert ist, nutzt 'python3' nun die Pakete aus der venv UND ROS Jazzy
python3 src/passform_agent_planning/scripts/socket_bridge.py &
BRIDGE_PID=$!
echo "✅ Socket-Bridge läuft (PID: $BRIDGE_PID)"

python3 src/passform_agent_planning/scripts/monitor_node.py &
MONITOR_PID=$!
echo "✅ Monitor-Node läuft (PID: $MONITOR_PID)"

cd "$FRONTEND_PATH" || exit
python3 test_agent.py &
TEST_AGENT_PID=$!
echo "✅ Test-Agenten aktiv (PID: $TEST_AGENT_PID)"

echo "------------------------------------------------"
echo "🖥️  System ist bereit. Öffne das Frontend im Browser."
echo "⌨️  Drücke Strg+C, um das System zu stoppen."
echo "------------------------------------------------"

# Script am Laufen halten
wait