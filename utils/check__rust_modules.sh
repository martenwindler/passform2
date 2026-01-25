#!/bin/sh

# Pfad relativ vom Projekt-Root ~/passform2
TARGET_DIR="passform2_agent_backend/src"
LIB_FILE="$TARGET_DIR/lib.rs"

if [ ! -d "$TARGET_DIR" ]; then
    echo "❌ Fehler: Verzeichnis '$TARGET_DIR' nicht gefunden."
    exit 1
fi

echo "🔍 Starte Rust-Architektur-Check (POSIX-safe)..."
echo "--------------------------------------------------"

# 1. Check: lib.rs Existenz
if [ ! -f "$LIB_FILE" ]; then
    echo "❌ FEHLT: src/lib.rs"
else
    echo "✅ OK:     src/lib.rs gefunden."
    
    echo "--- Prüfe Inhalte der lib.rs ---"
    for dir_path in "$TARGET_DIR"/*/; do
        dir_name=$(basename "$dir_path")
        
        # 'bin' ist ein Spezialordner für Binaries, kein Library-Modul!
        if [ "$dir_name" = "bin" ]; then continue; fi
        
        if grep -q "mod $dir_name;" "$LIB_FILE"; then
            echo "  ✅ Modul '$dir_name' ist in lib.rs angemeldet."
        else
            echo "  ⚠️  WARNUNG: '$dir_name' ist NICHT in lib.rs angemeldet!"
        fi
    done
fi

echo "--------------------------------------------------"
echo "--- Prüfe Untermodule auf mod.rs Dateien ---"

# 2. Check: Unterordner (ausgenommen bin/)
find "$TARGET_DIR" -mindepth 1 -type d ! -path "*/bin*" ! -path "*/.*" | while read -r dir; do
    if [ ! -f "$dir/mod.rs" ]; then
        echo "⚠️  FEHLT: $dir/mod.rs"
    else
        echo "✅ OK:     $dir"
        
        # Check: Sind die .rs Dateien im Ordner auch in der mod.rs angemeldet?
        for rs_file in "$dir"/*.rs; do
            file_name=$(basename "$rs_file" .rs)
            
            # mod.rs selbst und Dateien mit Sonderzeichen ignorieren
            if [ "$file_name" = "mod" ] || [ "$file_name" = "*" ]; then continue; fi
            
            if grep -q "mod $file_name;" "$dir/mod.rs"; then
                 : # Alles ok
            else
                 echo "     ↳ ⚠️  '$file_name.rs' fehlt in mod.rs!"
            fi
        done
    fi
done

echo "--------------------------------------------------"