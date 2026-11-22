#!/usr/bin/env bash
set -euo pipefail

workspace_root="${ROBOT_WS:-${HOME}/robot_ws}"
maps_dir="${MID360_MAPS_DIR:-$workspace_root/src/mid360_nav_bringup/maps}"
ros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"

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

if [ ! -r "$ros_setup" ] || [ ! -r "$workspace_root/install/setup.bash" ]; then
  echo "Error: ROS 2 or workspace setup is missing. See ros2_ws/README.md." >&2
  exit 2
fi
source_relaxed "$ros_setup"
source_relaxed "$workspace_root/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

map_name="${FASTLIO_MAP_NAME:-fastlio_map}"
if [ "$#" -gt 0 ] && [[ "$1" != -* ]]; then
  map_name="$1"
  shift
fi
if [[ ! "$map_name" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Error: invalid map name: $map_name" >&2
  exit 1
fi

mkdir -p "$maps_dir"
maps_dir="$(realpath -- "$maps_dir")"
map_base="$maps_dir/$map_name"
pcd_path="$map_base.pcd"
pcd2pgm_params="$workspace_root/src/mid360_nav_bringup/config/pcd2pgm.yaml"

if [ "$#" -ne 0 ]; then
  echo "Error: unexpected arguments: $*" >&2
  echo "pcd2pgm parameters are configured in $pcd2pgm_params" >&2
  exit 2
fi

if ! service_list="$(timeout 5 ros2 service list --no-daemon --spin-time 0.5)"; then
  echo "Error: could not discover the active ROS map save service." >&2
  exit 1
fi
if grep -qx /dual_map_save <<<"$service_list"; then
  map_mode=dual
  map_node=/dual_mid360_map
  map_service=/dual_map_save
elif grep -qx /map_save <<<"$service_list"; then
  map_mode=primary
  map_node=/laser_mapping
  map_service=/map_save
  if ! pcd_save_state="$(timeout 5 ros2 param get "$map_node" pcd_save.pcd_save_en --hide-type)" ||
     [ "$pcd_save_state" != "True" ]; then
    echo "Error: FAST-LIO PCD saving is disabled and the dual map saver is unavailable." >&2
    exit 1
  fi
else
  echo "Error: no active FAST-LIO or dual MID-360 map save service." >&2
  echo "Start mapping first with: just map $map_name" >&2
  exit 1
fi

if ! producer_path="$(timeout 5 ros2 param get "$map_node" map_file_path --hide-type)"; then
  echo "Error: could not read the output path from $map_node." >&2
  exit 1
fi
if [[ "$producer_path" != /* ]] ||
   [ "$(realpath -m -- "$(dirname -- "$producer_path")")/$(basename -- "$producer_path")" != "$pcd_path" ]; then
  echo "Error: $map_node is saving to '$producer_path', but this save expects '$pcd_path'." >&2
  echo "Use the same map name that started the mapping session." >&2
  exit 1
fi

if ! pcd2pgm_prefix="$(ros2 pkg prefix pcd2pgm 2>/dev/null)"; then
  echo "Error: upstream pcd2pgm is not installed in $workspace_root/install." >&2
  echo "Build $workspace_root/src/pcd2pgm before saving a map." >&2
  exit 1
fi
pcd2pgm_node="$pcd2pgm_prefix/lib/pcd2pgm/pcd2pgm_node"
if [ ! -x "$pcd2pgm_node" ]; then
  echo "Error: installed PCD converter executable is missing: $pcd2pgm_node" >&2
  exit 1
fi
if [ ! -r "$pcd2pgm_params" ]; then
  echo "Error: pcd2pgm parameter file is missing: $pcd2pgm_params" >&2
  exit 1
fi

# Lock the maps directory itself so a concurrent manual or automatic save
# cannot replace a set while another converter is still running.
exec 8<"$maps_dir"
if ! flock -n 8; then
  echo "Error: a map save is already in progress in $maps_dir." >&2
  exit 1
fi

map_files=("$pcd_path" "$map_base.pgm" "$map_base.yaml" "$map_base.lidar-mode")
for file in "${map_files[@]}"; do
  if [ -e "$file" ] && [ ! -f "$file" ] && [ ! -L "$file" ]; then
    echo "Error: map output is not a regular file: $file" >&2
    exit 1
  fi
done
stage_dir="$(mktemp -d -- "$maps_dir/.$map_name.save.XXXXXX")"
backup_dir="$stage_dir/previous"
staged_base="$stage_dir/$map_name"
map_topic="/pcd2pgm_map_save_${BASHPID}"
pcd2pgm_pid=""
backup_complete=0
committed=0

stop_pcd2pgm() {
  local attempt
  if [ -n "$pcd2pgm_pid" ] && kill -0 "$pcd2pgm_pid" 2>/dev/null; then
    kill -TERM "$pcd2pgm_pid" 2>/dev/null || true
    for ((attempt = 0; attempt < 20; attempt++)); do
      kill -0 "$pcd2pgm_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$pcd2pgm_pid" 2>/dev/null; then
      kill -KILL "$pcd2pgm_pid" 2>/dev/null || true
    fi
    wait "$pcd2pgm_pid" 2>/dev/null || true
  fi
}

cleanup() {
  stop_pcd2pgm
  local file backup restore_failed=0
  if [ "$committed" -eq 0 ]; then
    if [ "$backup_complete" -eq 1 ] && ! rm -f -- "${map_files[@]}"; then
      restore_failed=1
    fi
    for file in "${map_files[@]}"; do
      backup="$backup_dir/${file##*/}"
      if [ -e "$backup" ] || [ -L "$backup" ]; then
        mv -fT -- "$backup" "$file" || restore_failed=1
      fi
    done
  fi
  if [ "$restore_failed" -eq 0 ]; then
    rm -rf -- "$stage_dir"
  else
    echo "Error: map restoration incomplete. Remaining backup files are in $backup_dir." >&2
  fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
mkdir -- "$backup_dir"
# Both writers save to their launch-time path. Move the previous set out of
# the way so a missing or failed service write cannot reuse an old PCD.
for file in "${map_files[@]}"; do
  if [ -e "$file" ] || [ -L "$file" ]; then
    mv -- "$file" "$backup_dir/"
  fi
done
backup_complete=1

echo ">>> Saving $map_mode PCD from $map_node"
map_save_response="$(ros2 service call "$map_service" std_srvs/srv/Trigger '{}')"
printf '%s\n' "$map_save_response"
if [[ "$map_save_response" != *"success=True"* ]] || [ ! -s "$pcd_path" ]; then
  echo "Error: $map_service did not save a new PCD at $pcd_path." >&2
  exit 1
fi

echo ">>> Filtering PCD and publishing the occupancy grid with upstream pcd2pgm"
"$pcd2pgm_node" --ros-args \
  --params-file "$pcd2pgm_params" \
  -p pcd_file:="$pcd_path" \
  -p map_topic_name:="$map_topic" &
pcd2pgm_pid=$!

# pcd2pgm performs its PCL filters during node construction and publishes the
# resulting transient-local map once per second. Wait for the first complete
# map so map_saver_cli never races node startup on a large PCD.
if ! timeout 60 ros2 topic echo "$map_topic" nav_msgs/msg/OccupancyGrid \
    --once \
    --qos-reliability reliable \
    --qos-durability transient_local >/dev/null; then
  if ! kill -0 "$pcd2pgm_pid" 2>/dev/null; then
    wait "$pcd2pgm_pid" || true
  fi
  echo "Error: pcd2pgm did not publish $map_topic within 60 seconds." >&2
  exit 1
fi

echo ">>> Saving the filtered occupancy grid as Nav2 PGM/YAML"
ros2 run nav2_map_server map_saver_cli \
  -t "$map_topic" \
  -f "$staged_base" \
  --fmt pgm \
  --mode trinary \
  --ros-args -p save_map_timeout:=30.0

stop_pcd2pgm
pcd2pgm_pid=""

if [ ! -s "$staged_base.pgm" ] || [ ! -s "$staged_base.yaml" ]; then
  echo "Error: map_saver_cli did not create both the PGM and YAML files." >&2
  exit 1
fi
printf '%s\n' "$map_mode" > "$staged_base.lidar-mode"
for extension in pgm yaml lidar-mode; do
  mv -T -- "$staged_base.$extension" "$map_base.$extension"
done
committed=1

echo ">>> $map_mode map set is ready"
echo "    $pcd_path"
echo "    $map_base.pgm"
echo "    $map_base.yaml"
echo "    $map_base.lidar-mode"
echo "Navigate with: just nav $map_name"
