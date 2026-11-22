#!/usr/bin/env bash
set -euo pipefail

workspace_root="${ROBOT_WS:-${HOME}/robot_ws}"
maps_dir="${MID360_MAPS_DIR:-$workspace_root/src/mid360_nav_bringup/maps}"
ros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
stack_lock="${MID360_STACK_LOCK:-/tmp/nyu_mid360_mapping_navigation.lock}"

# The MID-360/FAST-LIO/TF chain is single-owner.  Sharing this lock with the
# navigation launcher prevents duplicate drivers and coordinate-frame trees.
exec 9>"$stack_lock"
if ! flock -n 9; then
  echo "Error: another MID-360 mapping/navigation stack is already running." >&2
  echo "Stop its terminal with Ctrl-C before starting a new stack." >&2
  exit 1
fi

source_relaxed() {
  local setup_file="$1"
  local restore_nounset=0
  case $- in
    *u*) restore_nounset=1; set +u ;;
  esac
  # shellcheck disable=SC1090
  source "$setup_file"
  if [ "$restore_nounset" -eq 1 ]; then
    set -u
  fi
}

configure_rviz_display() {
  if [ -n "${NAV_RVIZ_DISPLAY:-}" ]; then
    export DISPLAY="$NAV_RVIZ_DISPLAY"
    if [[ "$DISPLAY" == :* ]]; then
      export XAUTHORITY="${NAV_RVIZ_XAUTHORITY:-${HOME}/.Xauthority}"
    fi
    return
  fi

  # Prefer the NUC TigerVNC desktop (:2). Do not keep an SSH forwarded DISPLAY
  # such as localhost:10.0 — RViz would abort there instead of appearing in VNC.
  if [ -S /tmp/.X11-unix/X2 ] && [ -r "${HOME}/.vnc/sentry-Xauthority" ]; then
    export DISPLAY=:2
    export XAUTHORITY="${HOME}/.vnc/sentry-Xauthority"
    return
  fi

  if [ -S /tmp/.X11-unix/X1 ]; then
    export DISPLAY=:1
    export XAUTHORITY="${NAV_RVIZ_XAUTHORITY:-${HOME}/.Xauthority}"
    return
  fi

  if [ -n "${SSH_CONNECTION:-}" ]; then
    echo "Warning: SSH detected but no VNC display (:2 or :1) is available; RViz may not start." >&2
  fi
}

if [ ! -r "$ros_setup" ] || [ ! -r "$workspace_root/install/setup.bash" ]; then
  echo "Error: ROS 2 or workspace setup is missing. See ros2_ws/README.md." >&2
  exit 2
fi
source_relaxed "$ros_setup"
source_relaxed "$workspace_root/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
configure_rviz_display

node_list="$(timeout 3 ros2 node list --no-daemon --spin-time 0.2 2>/dev/null || true)"
if grep -Eq '^/(sentry_nav2_bridge|twist_mux)$' <<<"$node_list"; then
  echo "Error: a standalone chassis bridge or twist_mux is already running." >&2
  echo "Stop the current 'just wasd' terminal, start mapping, then run 'just wasd' again." >&2
  echo "The restarted WASD controller will reuse the mapping stack's bridge." >&2
  exit 1
fi

map_name="${FASTLIO_MAP_NAME:-fastlio_map}"
if [ "$#" -gt 0 ] && [[ "$1" != -* ]] && [[ "$1" != *":="* ]]; then
  map_name="$1"
  shift
fi
if [[ ! "$map_name" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Error: map name may contain only letters, digits, dot, underscore and dash." >&2
  exit 1
fi

pcd_path="$maps_dir/$map_name.pcd"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
save_map_script="$script_dir/save_map.sh"
mkdir -p "$maps_dir"
echo ">>> Starting MID-360 + FAST-LIO mapping"
echo "    PCD output: $pcd_path"
echo "    RViz display: ${DISPLAY:-not set}"
echo "    Drive with /cmd_vel_teleop; Nav2 is not active in mapping mode."
echo "    Ctrl-C saves $map_name (PCD + PGM/YAML) then stops the stack."
echo "    Skip the save with FASTLIO_MAP_AUTOSAVE=0. Manual save: just map-save $map_name"

launch_pid=""
stopping=0

stop_launch() {
  if [ -z "$launch_pid" ]; then
    return
  fi
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}

on_stop() {
  if [ "$stopping" -eq 1 ]; then
    return
  fi
  stopping=1
  trap '' INT TERM
  if [ -n "$launch_pid" ] && kill -0 "$launch_pid" 2>/dev/null \
      && [[ "${FASTLIO_MAP_AUTOSAVE:-1}" != "0" ]]; then
    echo ">>> Ctrl-C: saving map '$map_name' before shutdown"
    if ! "$save_map_script" "$map_name"; then
      echo "Warning: auto-save failed; shutting down mapping anyway." >&2
    fi
  fi
  echo ">>> Stopping mapping stack"
  stop_launch
  exit 0
}

trap on_stop INT TERM

# Own process group so the TTY Ctrl-C hits this wrapper first, not ros2 launch.
# Otherwise FAST-LIO dies before /map_save can write the PCD.
set -m
ros2 launch mid360_nav_bringup fastlio_mapping.launch.py \
  pcd_path:="$pcd_path" \
  "$@" &
launch_pid=$!
set +m

set +e
wait "$launch_pid"
launch_rc=$?
set -e
exit "$launch_rc"
