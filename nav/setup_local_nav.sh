#!/usr/bin/env bash

NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNDERLAY_SETUP="${NAV_UNDERLAY_SETUP:-/workspace/nav_ws/install/setup.bash}"

if [ ! -f "$UNDERLAY_SETUP" ]; then
    echo "[ERROR] Nav underlay not found: $UNDERLAY_SETUP" >&2
    return 1 2>/dev/null || exit 1
fi
if [ ! -f "$NAV_ROOT/install/setup.bash" ]; then
    echo "[ERROR] Local nav overlay is not built. Run $NAV_ROOT/build_local_nav.sh" >&2
    return 1 2>/dev/null || exit 1
fi

case "$-" in
    *u*) restore_nounset=1 ;;
    *) restore_nounset=0 ;;
esac
set +u
source "$UNDERLAY_SETUP"
source "$NAV_ROOT/install/setup.bash"
if [ "$restore_nounset" -eq 1 ]; then
    set -u
fi
unset restore_nounset
