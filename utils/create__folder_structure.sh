#!/bin/bash

# Wir gehen davon aus, dass wir im Hauptverzeichnis ~/passform2 sind
# Falls nicht, suchen wir den Ordner relativ zum Skript
TARGET_ROOT="passform2_agent_backend"

if [ ! -d "$TARGET_ROOT" ]; then
    echo "❌ Fehler: Ich kann den Ordner '$TARGET_ROOT' nicht finden."
    echo "Stelle sicher, dass du dich in ~/passform2 befindest."
    exit 1
fi

echo "🚀 Erstelle Struktur in $TARGET_ROOT/src..."

# Ordner erstellen
mkdir -p "$TARGET_ROOT/src/bin"
mkdir -p "$TARGET_ROOT/src/core/basyx"
mkdir -p "$TARGET_ROOT/src/core/skills"
mkdir -p "$TARGET_ROOT/src/core/types"
mkdir -p "$TARGET_ROOT/src/core/util"
mkdir -p "$TARGET_ROOT/src/managers"
mkdir -p "$TARGET_ROOT/src/ros"
mkdir -p "$TARGET_ROOT/launch"
mkdir -p "$TARGET_ROOT/config"
mkdir -p "$TARGET_ROOT/urdf"
mkdir -p "$TARGET_ROOT/meshes"

# Rust Module (mod.rs) erstellen
touch "$TARGET_ROOT/src/core/mod.rs"
touch "$TARGET_ROOT/src/core/basyx/mod.rs"
touch "$TARGET_ROOT/src/core/skills/mod.rs"
touch "$TARGET_ROOT/src/core/types/mod.rs"
touch "$TARGET_ROOT/src/core/util/mod.rs"
touch "$TARGET_ROOT/src/managers/mod.rs"
touch "$TARGET_ROOT/src/ros/mod.rs"

echo "✅ Alles erledigt! Struktur ist bereit."