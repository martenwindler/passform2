#!/bin/bash
# Passform2/utils/scripts/create__files_by_token.sh

# Check for --quiet / --q flag
QUIET=false
for arg in "$@"; do
    if [[ "$arg" == "--quiet" || "$arg" == "--q" ]]; then
        QUIET=true
        # Remove the quiet flag from the positional arguments
        set -- "${@/--quiet/}"
        set -- "${@/--q/}"
    fi
done

# Check args after removing quiet flag
if [ $# -ne 3 ]; then
    echo "Usage: $0 <prefix> <extension> <target_folder> [--quiet|--q]"
    exit 1
fi

new_prefix="$1"
file_ext="$2"
target_folder="$3"

mkdir -p "$target_folder"

tokens=()

# Read piped input and extract tokens
while IFS= read -r line; do
    if [[ "$line" =~ ^[[:space:]]*[├└]──[[:space:]]*([a-zA-Z0-9_.-]+)$ ]]; then
        file="${BASH_REMATCH[1]}"
        [[ "$file" == "__init__.py" ]] && continue

        base="${file%.*}"
        if [[ "$base" =~ .*__([^_][^_]*)$ ]]; then
            token="${BASH_REMATCH[1]}"
            tokens+=("$token")
        else
            if [ "$QUIET" = false ]; then
                echo "[Warning] Skipping file (no token found): $file"
            fi
        fi
    fi
done

if [ "${#tokens[@]}" -eq 0 ]; then
    echo "[Error] No valid tokens extracted."
    exit 1
fi

# Create files
for token in "${tokens[@]}"; do
    new_file="${target_folder}/${new_prefix}__${token}.${file_ext}"
    touch "$new_file"
    if [ "$QUIET" = false ]; then
        echo "Created: $new_file"
    fi
done
