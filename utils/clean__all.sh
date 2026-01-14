#!/bin/bash
set -e
# Passform2/utils/scripts/clean__all.sh

# Navigate to the project root (one level up from script location)
cd "$(dirname "$0")" || exit 1

# Detect --quiet or --q flag
QUIET_FLAG=""
for arg in "$@"; do
  if [[ "$arg" == "--quiet" || "$arg" == "--q" ]]; then
    QUIET_FLAG="--quiet"
    break
  fi
done

if [ -z "$QUIET_FLAG" ]; then
  echo "Current directory: $(pwd)"
  echo "Found files:"
  ls clean__*.sh
fi

# Run all clean__*.sh scripts except clean__all.sh
for script in clean__*.sh; do
  if [ "$script" != "clean__all.sh" ]; then
    [ -z "$QUIET_FLAG" ] && echo "Running $script"
    bash "$script" "$QUIET_FLAG"
  else
    [ -z "$QUIET_FLAG" ] && echo "Skipping $script"
  fi
done
