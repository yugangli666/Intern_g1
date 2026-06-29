#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# ros_foxy_env.sh — independent ROS 2 Foxy environment for Unitree G1
#
# Usage (to be sourced, NOT executed):
#   source ros_foxy_env.sh
#   source_g1_ros_foxy
#
# Overridable environment variables:
#   ROS_SETUP            path to foxy setup.bash (default: /opt/ros/foxy/setup.bash)
#   UNITREE_ROS_SETUP    path to Unitree CycloneDDS overlay (default:
#                        /home/unitree/unitree_ros2/cyclonedds_ws/install/local_setup.bash)
#   UNITREE_ROS_DOMAIN_ID  ROS domain ID (default: 0)
#
# This script deliberately does NOT source:
#   - /opt/ros/humble/setup.bash
#   - ~/unitree_ros2/setup.sh          (mixes in Noetic paths)
#   - ~/ros2_ws/install/setup.bash     (Humble workspace)
# ---------------------------------------------------------------------------

# ── remove_ros_distribution_paths ────────────────────────────────────────
# Strip every entry under /opt/ros/* from common environment variables,
# then unset all ROS-distribution-level variables so that a subsequent
# source of a different distro starts from a clean slate.
# ─────────────────────────────────────────────────────────────────────────
remove_ros_distribution_paths() {
  local _var _tmp _IFS_save

  for _var in PATH LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH; do
    _tmp="${!_var:-}"
    if [ -z "$_tmp" ]; then
      continue
    fi
    _IFS_save="$IFS"
    IFS=:
    set -- $_tmp
    IFS="$_IFS_save"
    _tmp=""
    for _entry; do
      case "$_entry" in
        /opt/ros/*) continue ;;
      esac
      if [ -z "$_tmp" ]; then
        _tmp="$_entry"
      else
        _tmp="${_tmp}:${_entry}"
      fi
    done
    export "$_var"="$_tmp"
  done

  # Clear ROS-distribution-level variables
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX
  unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  unset ROS_PACKAGE_PATH ROS_ROOT ROS_ETC_DIR ROS_MASTER_URI
}

# ── source_g1_ros_foxy ──────────────────────────────────────────────────
# Source ROS 2 Foxy and the Unitree CycloneDDS overlay, then validate.
# Call AFTER remove_ros_distribution_paths if another distro may already
# be sourced in the current shell.
# ─────────────────────────────────────────────────────────────────────────
source_g1_ros_foxy() {
  local ros_setup unitree_setup domain_id
  local _nounset_was_active

  ros_setup="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
  unitree_setup="${UNITREE_ROS_SETUP:-/home/unitree/unitree_ros2/cyclonedds_ws/install/local_setup.bash}"
  domain_id="${UNITREE_ROS_DOMAIN_ID:-0}"

  # If set -u is active, Foxy's setup.bash will fail because it reads
  # optional variables without guarding them.  Temporarily allow unset.
  case "${-//[^u]/}" in
    u) _nounset_was_active=1 ; set +u ;;
    *) _nounset_was_active=0 ;;
  esac

  # 1. Remove any previous ROS distribution paths
  remove_ros_distribution_paths

  # 2. Source Foxy
  if [ ! -f "$ros_setup" ]; then
    echo "[ros_foxy_env] ERROR: ROS 2 Foxy setup not found: $ros_setup" >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "$ros_setup"

  # 3. Source Unitree CycloneDDS overlay
  if [ ! -f "$unitree_setup" ]; then
    echo "[ros_foxy_env] ERROR: Unitree overlay not found: $unitree_setup" >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "$unitree_setup"

  # Re-enable nounset if it was active
  if [ "$_nounset_was_active" -eq 1 ]; then
    set -u
  fi

  # 4. Set CycloneDDS as the RMW implementation
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID="$domain_id"

  # 5. Validate the environment
  local _errors=0

  if [ "${ROS_DISTRO:-}" != "foxy" ]; then
    echo "[ros_foxy_env] ERROR: ROS_DISTRO is '${ROS_DISTRO:-<unset>}', expected 'foxy'" >&2
    _errors=1
  fi

  if ! ros2 pkg prefix unitree_api >/dev/null 2>&1; then
    echo "[ros_foxy_env] ERROR: unitree_api package not found after sourcing Foxy + Unitree overlay" >&2
    _errors=1
  fi

  if ! ros2 pkg prefix unitree_go >/dev/null 2>&1; then
    echo "[ros_foxy_env] ERROR: unitree_go package not found after sourcing Foxy + Unitree overlay" >&2
    _errors=1
  fi

  if [ "$_errors" -ne 0 ]; then
    echo "[ros_foxy_env] Environment validation FAILED. Check the paths above." >&2
    return 1
  fi

  echo "[ros_foxy_env] ROS_DISTRO=$ROS_DISTRO  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION  DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "[ros_foxy_env] unitree_api: $(ros2 pkg prefix unitree_api)"
  echo "[ros_foxy_env] unitree_go:  $(ros2 pkg prefix unitree_go)"
  return 0
}
