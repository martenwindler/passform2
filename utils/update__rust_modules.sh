#!/bin/sh

# Pfad zum src-Ordner
TARGET_DIR="passform2_agent_backend/src"

if [ ! -d "$TARGET_DIR" ]; then
    echo "❌ Fehler: Verzeichnis '$TARGET_DIR' nicht gefunden."
    exit 1
fi

echo "🔧 Starte automatische Modul-Reparatur..."

# Finde alle Unterordner (außer bin/ und versteckte)
find "$TARGET_DIR" -mindepth 1 -type d ! -path "*/bin*" ! -path "*/.*" | while read -r dir; do
    MOD_FILE="$dir/mod.rs"
    
    echo "📂 Verarbeite Ordner: $dir"
    
    # Header für die mod.rs schreiben
    echo "// Automatisch generiert durch fix__rust_modules.sh" > "$MOD_FILE"
    
    # Alle .rs Dateien im Ordner finden (außer mod.rs selbst)
    for rs_path in "$dir"/*.rs; do
        file_name=$(basename "$rs_path" .rs)
        
        if [ "$file_name" = "mod" ] || [ "$file_name" = "*" ]; then
            continue
        fi
        
        # In mod.rs eintragen
        echo "pub mod $file_name;" >> "$MOD_FILE"
        echo "  ➕ angemeldet: $file_name.rs"
    done
done

echo "--------------------------------------------------"
echo "✅ Alle mod.rs Dateien wurden aktualisiert!"
echo "Tipp: Führe jetzt 'sh utils/check__rust_modules.sh' zur Kontrolle aus."