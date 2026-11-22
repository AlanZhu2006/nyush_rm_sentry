#!/usr/bin/env bash
set -eo pipefail

directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project="$(cd -- "$directory/../.." && pwd)"
export ROBOT_WS="${ROBOT_WS:-$HOME/robot_ws}"
ros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
if [[ ! -r "$ros_setup" || ! -r "$ROBOT_WS/install/setup.bash" ]]; then
  printf 'ROS 2 or the robot workspace setup is missing in %s\n' "$ROBOT_WS" >&2
  exit 2
fi
source "$ros_setup"
source "$ROBOT_WS/install/setup.bash"

bridge_prefix="${FOXGLOVE_BRIDGE_PREFIX:-$HOME/.local/opt/sentry-foxglove/opt/ros/$ROS_DISTRO}"
if [[ -x "$bridge_prefix/lib/foxglove_bridge/foxglove_bridge" ]]; then
  export AMENT_PREFIX_PATH="$bridge_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$bridge_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  printf 'Install ros-%s-foxglove-bridge or set FOXGLOVE_BRIDGE_PREFIX to its ROS prefix.\n' "$ROS_DISTRO" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY=1
export SENTRY_OPERATOR_PYTHON="${FOXGLOVE_PYTHON:-$project/.venv/bin/python}"
if [[ ! -x "$SENTRY_OPERATOR_PYTHON" ]]; then
  export SENTRY_OPERATOR_PYTHON=/usr/bin/python3
fi
export PYTHONPATH="$project:${PYTHONPATH:+$PYTHONPATH:}/usr/lib/python3/dist-packages"
export PYTHONDONTWRITEBYTECODE=1
if ! "$SENTRY_OPERATOR_PYTHON" -c 'from websockets.asyncio.client import connect' >/dev/null 2>&1; then
  printf 'The Foxglove backend requires websockets 15 or later in its Python environment.\n' >&2
  exit 2
fi
exec ros2 launch "$directory/console.launch.py"
