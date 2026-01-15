#!/bin/bash
# Passform2/utils/scripts/clean__zone-identifier.sh

# Navigate to the project root
cd "$(dirname "$0")/.."

# Parse quiet flag
QUIET=false
for arg in "$@"; do
    if [[ "$arg" == "--quiet" || "$arg" == "--q" ]]; then
        QUIET=true
        break
    fi
done

if [ "$QUIET" = false ]; then
    echo "Cleaning all files ending with 'Zone.Identifier'..."
fi

# Find and delete all matching files
find . -type f -name '*Zone.Identifier' -print -delete

if [ "$QUIET" = false ]; then
    echo "Cleanup complete."
fi