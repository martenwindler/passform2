#!/bin/bash
# Passform2/utils/print__folder_contents_by_depth_w_files.sh

usage() {
  echo "Usage: $0 <directory> [max_depth]"
  exit 1
}

ROOT="$1"
MAX_DEPTH="${2:-99}"

# Validate input
[[ -z "$ROOT" || ! -d "$ROOT" ]] && usage

ROOT=$(realpath "$ROOT")

print_tree() {
  local dir="$1"
  local prefix="$2"
  local current_depth="$3"
  local max_depth="$4"

  # Only recurse if we haven't reached max depth
  if (( current_depth > max_depth )); then
    return
  fi

  # Get sorted list of entries
  local entries=()
  while IFS= read -r -d '' entry; do
    entries+=("$entry")
  done < <(find "$dir" -mindepth 1 -maxdepth 1 -print0 | sort -z)

  local total=${#entries[@]}

  for ((i = 0; i < total; i++)); do
    local path="${entries[$i]}"
    local name=$(basename "$path")
    local is_last=$(( i == total - 1 ))
    local connector="├── "
    [[ "$is_last" == 1 ]] && connector="└── "

    if [[ -d "$path" && ! -L "$path" ]]; then
      echo "${prefix}${connector}${name}/"
      local next_prefix="$prefix"
      [[ "$is_last" == 1 ]] && next_prefix+="    " || next_prefix+="│   "
      print_tree "$path" "$next_prefix" $((current_depth + 1)) "$max_depth"
    elif [[ -f "$path" ]]; then
      echo "${prefix}${connector}${name}"
    fi
  done
}

# === Output Root ===
echo "$(basename "$ROOT")/"
print_tree "$ROOT" "" 1 "$MAX_DEPTH"
