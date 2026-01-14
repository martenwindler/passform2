#!/bin/bash
# Passform2/utils/print__folder_contents_by_depth.sh

# Usage info
usage() {
  echo "Usage: $0 <directory> [max_depth]"
  exit 1
}

# Arguments
ROOT="$1"
MAX_DEPTH="$2"

if [[ -z "$ROOT" ]]; then
  usage
fi

if [[ ! -d "$ROOT" ]]; then
  echo "❌ '$ROOT' is not a valid directory."
  exit 1
fi

# Normalize root path and depth
ROOT=$(realpath "$ROOT")
[[ -z "$MAX_DEPTH" ]] && MAX_DEPTH=99

# Detect depth of root path
ROOT_DEPTH=$(awk -F'/' '{print NF}' <<< "$ROOT")

# Traverse directories
find "$ROOT" -type d | while read -r dir; do
  CURRENT_DEPTH=$(awk -F'/' '{print NF}' <<< "$dir")
  REL_DEPTH=$((CURRENT_DEPTH - ROOT_DEPTH))

  # Skip if too deep
  if (( REL_DEPTH > MAX_DEPTH )); then
    continue
  fi

  # Indentation based on depth
  indent=""
  for ((i=0; i<REL_DEPTH; i++)); do
    indent+="│   "
  done

  # Connector symbol
  base=$(basename "$dir")
  echo "${indent}├── ${base}/"
done
