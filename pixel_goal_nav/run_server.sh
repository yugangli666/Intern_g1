#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PYTHON_BIN="${PYTHON_BIN:-python}"
exec "$PYTHON_BIN" pixel_goal_nav/server.py \
  --port "${PIXEL_GOAL_PORT:-5802}" \
  --model-path "${PIXEL_GOAL_MODEL_PATH:-checkpoints/InternVLA-N1-DualVLN}" \
  "$@"
