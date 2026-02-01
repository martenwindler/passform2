#!/usr/bin/env bash

# Bestimme das Projektverzeichnis absolut sicher
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "🛡️  Repariere Berechtigungen in: $PROJECT_ROOT"

# 1. Verzeichnisse auf 755 (außer install/build/node_modules/target)
find "$PROJECT_ROOT" \
    \( -path "*/install" -o -path "*/build" -o -path "*/node_modules" -o -path "*/target" -o -path "*/.git" \) -prune \
    -o -type d -exec chmod 755 {} +

# 2. Dateien auf 644
find "$PROJECT_ROOT" \
    \( -path "*/install" -o -path "*/build" -o -path "*/node_modules" -o -path "*/target" -o -path "*/.git" \) -prune \
    -o -type f -exec chmod 644 {} +

# 3. Skripte und Rust-Binaries explizit auf 755 setzen
find "$PROJECT_ROOT" -name "*.sh" -exec chmod +x {} +
[ -d "$PROJECT_ROOT/passform2_agent_backend/src/bin" ] && chmod -R +x "$PROJECT_ROOT/passform2_agent_backend/src/bin"

echo "✅ Berechtigungen im Projekt sind nun korrekt gesetzt."