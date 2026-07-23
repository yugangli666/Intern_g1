#!/usr/bin/env bash

set -euo pipefail

NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNDERLAY_SETUP="${NAV_UNDERLAY_SETUP:-/workspace/nav_ws/install/setup.bash}"

if [ ! -f "$UNDERLAY_SETUP" ]; then
    echo "[ERROR] Nav underlay not found: $UNDERLAY_SETUP" >&2
    exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
    echo "[ERROR] colcon is not available" >&2
    exit 1
fi

# The underlay supplies GLIM, map localization, Nav2 plugins, and message dependencies.
# Generated colcon setup files probe optional variables and are not nounset-safe.
set +u
source "$UNDERLAY_SETUP"
set -u

colcon --log-base "$NAV_ROOT/log" build \
    --base-paths \
        "$NAV_ROOT/control_bridge" \
        "$NAV_ROOT/navigation_interfaces" \
        "$NAV_ROOT/nav2_simple_commander" \
        "$NAV_ROOT/nav2_bringup" \
    --build-base "$NAV_ROOT/build" \
    --install-base "$NAV_ROOT/install" \
    --symlink-install \
    --packages-select \
        control_bridge \
        navigation_interfaces \
        nav2_simple_commander \
        nav2_bringup \
    --event-handlers console_cohesion+

set +u
source "$NAV_ROOT/install/setup.bash"
set -u

for package in control_bridge navigation_interfaces nav2_simple_commander nav2_bringup; do
    prefix="$(ros2 pkg prefix "$package")"
    case "$prefix" in
        "$NAV_ROOT/install"/*)
            printf '[OK] %-24s %s\n' "$package" "$prefix"
            ;;
        *)
            echo "[ERROR] $package resolved outside the local overlay: $prefix" >&2
            exit 1
            ;;
    esac
done
