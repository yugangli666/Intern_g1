#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_CLIENT_DIR="${G1_CLIENT_DIR:-$SCRIPT_DIR/../g1_client}"

if [ -f "$G1_CLIENT_DIR/ros_foxy_env.sh" ]; then
  # shellcheck disable=SC1090
  source "$G1_CLIENT_DIR/ros_foxy_env.sh"
  source_g1_ros_foxy
fi

if [ -f "$G1_CLIENT_DIR/dds_interface.sh" ]; then
  # shellcheck disable=SC1090
  source "$G1_CLIENT_DIR/dds_interface.sh"
  configure_cyclonedds_interface
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
cd "$SCRIPT_DIR"
exec "$PYTHON_BIN" g1_client.py "$@"
