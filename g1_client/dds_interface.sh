#!/usr/bin/env bash

resolve_dds_interface() {
  local requested="${UNITREE_DDS_INTERFACE:-}"
  if [ -z "$requested" ] && [[ "${CYCLONEDDS_URI:-}" =~ name=\"([^\"]+)\" ]]; then
    requested="${BASH_REMATCH[1]}"
  fi

  if [ -n "$requested" ] && [ -d "/sys/class/net/$requested" ]; then
    printf '%s\n' "$requested"
    return 0
  fi

  if [ -n "$requested" ]; then
    local alias_match
    alias_match="$(
      ip -o link show 2>/dev/null | awk -v wanted="$requested" '
        {
          iface = $2
          sub(/:$/, "", iface)
          for (i = 3; i <= NF; i++) {
            if ($i == "altname" && $(i + 1) == wanted) {
              print iface
              exit
            }
          }
        }
      '
    )"
    if [ -n "$alias_match" ] && [ -d "/sys/class/net/$alias_match" ]; then
      printf '%s\n' "$alias_match"
      return 0
    fi
  fi

  local unitree_subnet_match
  unitree_subnet_match="$(
    ip -o -4 addr show up scope global 2>/dev/null | awk '$4 ~ /^192\.168\.123\./ { print $2; exit }'
  )"
  if [ -n "$unitree_subnet_match" ]; then
    printf '%s\n' "$unitree_subnet_match"
    return 0
  fi

  ip -o -4 addr show up scope global 2>/dev/null | awk '{ print $2; exit }'
}

configure_cyclonedds_interface() {
  local dds_interface
  dds_interface="$(resolve_dds_interface)"
  if [ -n "$dds_interface" ]; then
    export UNITREE_DDS_INTERFACE="$dds_interface"
    # ROS 2 Foxy's bundled CycloneDDS rejects the newer
    # General/Interfaces/NetworkInterface XML syntax.  The legacy
    # NetworkInterfaceAddress form works with Foxy and later releases.
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>$dds_interface</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
    echo "Using CycloneDDS interface: $dds_interface"
  else
    echo "Warning: no active DDS network interface found; keeping existing CycloneDDS config." >&2
  fi
}
