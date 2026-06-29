#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Load Foxy environment (handles nounset internally) ─────────────────
# shellcheck disable=SC1091
source "$SCRIPT_DIR/ros_foxy_env.sh"
source_g1_ros_foxy

# ── CycloneDDS network-interface selection ─────────────────────────────
# shellcheck disable=SC1091
source "$SCRIPT_DIR/dds_interface.sh"
configure_cyclonedds_interface

# ── Launch the InternNav G1 client ─────────────────────────────────────
PYTHON_BIN="${PYTHON_BIN:-python3}"
cd "$SCRIPT_DIR"
exec "$PYTHON_BIN" "$SCRIPT_DIR/http_internvla_client_g1.py" "$@"
