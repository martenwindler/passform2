#!/bin/bash
# Passform2/utils/clean____pycache__.sh

# Navigate to the project root (directory of the script's parent)
cd "$(dirname "$0")/.." || exit 1

# Check for --quiet or --q flag
QUIET=false
for arg in "$@"; do
    if [[ "$arg" == "--quiet" || "$arg" == "--q" ]]; then
        QUIET=true
        break
    fi
done

if [ "$QUIET" = false ]; then
    echo "Cleaning folders ending with '__pycache__'."
fi

# Find and remove all __pycache__ folders in the project
find . -type d -name "__pycache__" -exec rm -rf {} +

if [ "$QUIET" = false ]; then
    echo "Cleanup complete."
fi
