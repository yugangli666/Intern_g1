#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$HOME/unitree_ros2/setup.sh" ]; then
  # shellcheck disable=SC1091
  source "$HOME/unitree_ros2/setup.sh"
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
fi

python3 "$SCRIPT_DIR/http_internvla_client_g1.py" "$@"
