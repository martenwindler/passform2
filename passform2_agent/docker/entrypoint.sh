#!/bin/bash
set -e

# ROS Setup
source /opt/ros/rolling/setup.bash
source /ros_ws/install/setup.bash

echo "=== ROS Agent Container Entrypoint ==="
echo "MODULE_TYPE: $MODULE_TYPE"
echo "POSITION_X: $POSITION_X" 
echo "POSITION_Y: $POSITION_Y"
echo "SKIP_NFC: $SKIP_NFC"

# Standard-Werte setzen
if [ -z "$MODULE_TYPE" ]; then
    MODULE_TYPE="greifer"
    echo "Verwende Standard MODULE_TYPE: $MODULE_TYPE"
fi

# Position bestimmen
if [ -n "$POSITION_X" ] && [ -n "$POSITION_Y" ]; then
    # Position manuell gesetzt
    echo "Position manuell gesetzt: [$POSITION_X, $POSITION_Y]"
    POSITION="[$POSITION_X, $POSITION_Y]"
    
elif [ "$SKIP_NFC" = "true" ]; then
    # NFC überspringen, Standard-Position verwenden
    echo "NFC wird übersprungen, verwende Standard-Position [0, 0]"
    POSITION="[0, 0]"
    
else
    # Position von NFC-Reader lesen
    echo "Lese Position vom NFC-Reader..."
    
    if python3 /nfc_reader.py > /tmp/nfc_output.txt 2>&1; then
        # NFC erfolgreich gelesen
        NFC_RESULT=$(tail -n 1 /tmp/nfc_output.txt)
        echo "NFC-Reader Ausgabe: $NFC_RESULT"
        
        # Parse x,y Format
        if [[ $NFC_RESULT =~ ^[0-9]+,[0-9]+$ ]]; then
            IFS=',' read -r POSITION_X POSITION_Y <<< "$NFC_RESULT"
            POSITION="[$POSITION_X, $POSITION_Y]"
            echo "Position vom NFC gelesen: $POSITION"
        else
            echo "Fehler: Ungültiges Format vom NFC-Reader"
            echo "NFC-Reader Log:"
            cat /tmp/nfc_output.txt
            echo "Verwende Standard-Position [0, 0]"
            POSITION="[0, 0]"
        fi
    else
        echo "Fehler beim Lesen des NFC-Chips"
        echo "NFC-Reader Log:"
        cat /tmp/nfc_output.txt
        echo "Verwende Standard-Position [0, 0]"
        POSITION="[0, 0]"
    fi
fi

echo "=== Starte ROS Agent ==="
echo "Module Type: $MODULE_TYPE"
echo "Position: $POSITION"

# ROS Launch starten
exec ros2 launch passform_agent_planning single_agent.launch.py \
    module_type:="$MODULE_TYPE" \
    position:="$POSITION"
