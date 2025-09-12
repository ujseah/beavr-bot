#!/usr/bin/env bash
set -euo pipefail

# rag.sh — Generate a quick code structure snapshot for BeaVR teleop
# Usage:
#   ./rag.sh [TELEOP_DIR] [OUTPUT_FILE]
# Defaults:
#   TELEOP_DIR=/home/demo/beavr-bot/src/beavr/teleop
#   OUTPUT_FILE=/home/demo/beavr-bot/codemap_teleop.txt

TELEOP_DIR=${1:-/home/demo/beavr-bot/src/beavr/teleop}
OUTPUT_FILE=${2:-/home/demo/beavr-bot/codemap_teleop.txt}

if [[ ! -d "$TELEOP_DIR" ]]; then
  echo "Teleop directory not found: $TELEOP_DIR" >&2
  exit 1
fi

echo "Generating code structure snapshot for: $TELEOP_DIR" >&2
{
  echo "=== BeaVR Teleop Code Structure ==="
  date
  echo
  echo "[ROOT]: $TELEOP_DIR"
  echo
  echo "--- Directory Tree (filtered) ---"
  # Requires 'tree'. If missing, suggest install
  if command -v tree >/dev/null 2>&1; then
    tree -a -I "__pycache__|*.pyc|*.pyo|*.so|*.dll|*.dylib|node_modules|dist|build|.git|.venv|*.egg-info" "$TELEOP_DIR"
  else
    echo "(tree not found; install with: sudo apt-get install tree)"
    find "$TELEOP_DIR" -type d -printf "%p\n" | sort
  fi
  echo

  echo "--- Python Modules: classes and functions ---"
  if command -v rg >/dev/null 2>&1; then
    rg -n --no-heading -H -e '^\s*class\s+\w+' -e '^\s*def\s+\w+' "$TELEOP_DIR" \
      | sed 's/^/  /' \
      | sort
  else
    echo "(ripgrep not found; install with: sudo apt-get install ripgrep)"
    grep -RInE '^\s*(class|def)\s+\w+' "$TELEOP_DIR" | sed 's/^/  /' | sort || true
  fi
  echo

  echo "--- Imports referencing utils ---"
  if command -v rg >/dev/null 2>&1; then
    rg -n --no-heading -H -e 'from\s+beavr\.teleop\.utils' -e 'import\s+.*utils' "$TELEOP_DIR" \
      | sed 's/^/  /' \
      | sort || true
  else
    grep -RInE 'from\s+beavr\.teleop\.utils|import\s+.*utils' "$TELEOP_DIR" | sed 's/^/  /' | sort || true
  fi
  echo

  echo "--- File sizes (top 30) ---"
  if command -v du >/dev/null 2>&1; then
    du -ah "$TELEOP_DIR" \
      | grep -E '\.py$' \
      | sort -hr \
      | head -n 30 \
      | sed 's/^/  /'
  fi
  echo
} > "$OUTPUT_FILE"

echo "Wrote: $OUTPUT_FILE" >&2


