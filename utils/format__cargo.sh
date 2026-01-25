#!/bin/sh

# Wir nehmen einfach das aktuelle Verzeichnis
BASE_DIR=$(pwd)

# Sicherheitscheck: Sind wir im passform2-Projekt?
if [ ! -d "$BASE_DIR/passform2_agent_backend" ] || [ ! -d "$BASE_DIR/passform2_ws" ]; then
    echo "❌ FEHLER: Bitte starte das Skript direkt aus dem Verzeichnis ~/passform2/"
    echo "Aktuelles Verzeichnis: $BASE_DIR"
    exit 1
fi

echo "🧹 Starte Deep Clean (Sicherheitsmodus)..."
echo "--------------------------------------------------"

# 1. RENAME: cargo.toml -> Cargo.toml
# Wir suchen nur 3 Ebenen tief, um nicht in Systemordner zu rutschen
echo "📝 Korrigiere Cargo.toml Schreibweise..."
find . -maxdepth 4 -name "cargo.toml" -type f | while read -r file; do
    mv "$file" "$(dirname "$file")/Cargo.toml"
    echo "  ✅ Fixed: $file"
done