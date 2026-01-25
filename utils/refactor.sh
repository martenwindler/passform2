#!/bin/bash

# Pfad-Definitionen
BACKEND_DIR="passform2_agent_backend"
APP_DIR="$BACKEND_DIR/app"
CORE_DIR="$APP_DIR/core"
SRC_DIR="$BACKEND_DIR/src"

echo "🚚 Starte großen Datei-Umzug in die Rust-Struktur..."

# Hilfsfunktion zum sicheren Verschieben
move_rs() {
    if [ -f "$1" ]; then
        mv "$1" "$2"
        echo "  ✅ Moved: $(basename "$1") -> $2"
    fi
}

# --- 1. BINARIES (Nodes mit main()) ---
echo "📂 Sortiere Binaries (src/bin)..."
move_rs "$CORE_DIR/module_base/module_base/inventory_manager.rs" "$SRC_DIR/bin/"
move_rs "$CORE_DIR/module_base/module_base/module_manager.rs"    "$SRC_DIR/bin/"
move_rs "$CORE_DIR/passform_skills/passform_skills/task_manager.rs" "$SRC_DIR/bin/"
move_rs "$CORE_DIR/passform_product/passform_product/product_publisher.rs" "$SRC_DIR/bin/"
move_rs "$APP_DIR/managers/nfc_manager.rs" "$SRC_DIR/bin/"
move_rs "$APP_DIR/managers/socket_io_manager.rs" "$SRC_DIR/bin/"

# --- 2. CORE LOGIC (Library) ---
echo "📂 Sortiere Core-Logik (src/core)..."

# Types
move_rs "$CORE_DIR/module_base/module_base/inventory.rs" "$SRC_DIR/core/types/"
# Alle Files aus passform_util/types/
if [ -d "$CORE_DIR/passform_util/passform_util/types" ]; then
    find "$CORE_DIR/passform_util/passform_util/types" -name "*.rs" -exec mv {} "$SRC_DIR/core/types/" \;
    echo "  ✅ Moved all types from passform_util"
fi

# Utils
move_rs "$CORE_DIR/passform_util/passform_util/helper.rs" "$SRC_DIR/core/util/"
move_rs "$CORE_DIR/passform_util/passform_util/log_level.rs" "$SRC_DIR/core/util/"
move_rs "$CORE_DIR/passform_util/passform_util/network.rs" "$SRC_DIR/core/util/"
move_rs "$CORE_DIR/passform_util/passform_util/registration.rs" "$SRC_DIR/core/util/"
move_rs "$CORE_DIR/passform_util/passform_util/time.rs" "$SRC_DIR/core/util/"
move_rs "$CORE_DIR/passform_util/passform_util/watchdog.rs" "$SRC_DIR/core/util/"

# Skills
move_rs "$CORE_DIR/passform_skills/passform_skills/base.rs" "$SRC_DIR/core/skills/"
move_rs "$CORE_DIR/passform_skills/passform_skills/mixins.rs" "$SRC_DIR/core/skills/"
move_rs "$CORE_DIR/passform_skills/passform_skills/relay.rs" "$SRC_DIR/core/skills/"

# Basyx
move_rs "$CORE_DIR/passform_util/passform_util/basyx/connector.rs" "$SRC_DIR/core/basyx/"

# --- 3. MANAGERS & ROS ---
echo "📂 Sortiere Manager & ROS (src/managers / src/ros)..."
move_rs "$APP_DIR/managers/agent_manager.rs" "$SRC_DIR/managers/"
move_rs "$APP_DIR/managers/config_manager.rs" "$SRC_DIR/managers/"
move_rs "$APP_DIR/managers/node_manager.rs"   "$SRC_DIR/managers/"
move_rs "$APP_DIR/managers/path_manager.rs"   "$SRC_DIR/managers/"
move_rs "$APP_DIR/managers/skill_manager.rs"  "$SRC_DIR/managers/"
move_rs "$APP_DIR/ros/ros_client.rs"          "$SRC_DIR/ros/"
move_rs "$APP_DIR/logic/planner.rs"           "$SRC_DIR/core/" # Oder eigenen Ordner erstellen

# --- 4. TOP-LEVEL FILES ---
echo "📂 Kopiere Hauptdateien..."
move_rs "$APP_DIR/main.rs" "$SRC_DIR/main.rs"
move_rs "$APP_DIR/lib.rs"  "$SRC_DIR/lib.rs"
move_rs "$APP_DIR/config.rs" "$SRC_DIR/config.rs"
move_rs "$APP_DIR/system_api.rs" "$SRC_DIR/system_api.rs"

# --- 5. RESSOURCEN (YAML, URDF, MESHES) ---
echo "📂 Sammle Ressourcen (config, urdf, meshes)..."
find "$CORE_DIR" -name "*.yaml" -exec cp {} "$BACKEND_DIR/config/" \;
find "$CORE_DIR" -name "*.xacro" -exec cp {} "$BACKEND_DIR/urdf/" \;
find "$CORE_DIR" -name "*.stl" -exec cp {} "$BACKEND_DIR/meshes/" \;

echo "✨ Refactoring abgeschlossen! Prüfe jetzt 'src/bin' und 'src/core'."