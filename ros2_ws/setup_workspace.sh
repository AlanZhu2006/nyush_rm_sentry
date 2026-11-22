#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "$script_dir/.." && pwd)"
workspace_root="${ROBOT_WS:-${HOME}/robot_ws}"
workspace_src="$workspace_root/src"
manifest="$script_dir/dependencies.repos"
fast_lio_patch="$script_dir/patches/fast_lio_mid360.patch"
dependencies=(FAST_LIO_ROS2 Livox-SDK2 livox_ros_driver2 pcd2pgm)
project_packages=(mid360_nav_bringup icp_probe)

for command_name in git vcs; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Error: required command is missing: $command_name" >&2
    exit 2
  fi
done

manifest_value() {
  local repository_name="$1"
  local field_name="$2"
  awk -v repository_name="$repository_name" -v field_name="$field_name" '
    $0 ~ "^  " repository_name ":$" { in_repository = 1; next }
    in_repository && $0 ~ "^    " field_name ":" { print $2; exit }
    in_repository && $0 ~ "^  [^ ]" { exit }
  ' "$manifest"
}

mkdir -p "$workspace_src"
vcs import --skip-existing "$workspace_src" < "$manifest"

for dependency in "${dependencies[@]}"; do
  dependency_dir="$workspace_src/$dependency"
  expected_commit="$(manifest_value "$dependency" version)"
  expected_url="$(manifest_value "$dependency" url)"
  if [ ! -d "$dependency_dir/.git" ]; then
    echo "Error: dependency was not imported: $dependency_dir" >&2
    exit 1
  fi
  actual_commit="$(git -C "$dependency_dir" rev-parse HEAD)"
  actual_url="$(git -C "$dependency_dir" remote get-url origin)"
  if [ "$actual_commit" != "$expected_commit" ]; then
    echo "Error: $dependency is at $actual_commit, expected $expected_commit." >&2
    echo "Refusing to change an existing checkout automatically." >&2
    exit 1
  fi
  if [ "${actual_url%.git}" != "${expected_url%.git}" ]; then
    echo "Error: $dependency origin is $actual_url, expected $expected_url." >&2
    exit 1
  fi
done

fast_lio_dir="$workspace_src/FAST_LIO_ROS2"
# vcstool does not populate Git submodules on a fresh import. Use the gitlink
# recorded by the pinned parent commit, never the submodule branch tip.
submodule_status="$(git -C "$fast_lio_dir" submodule status --recursive)"
if printf '%s\n' "$submodule_status" | grep -Eq '^[+U]'; then
  echo 'Error: an existing FAST-LIO submodule is at an unexpected commit.' >&2
  exit 1
fi
git -C "$fast_lio_dir" submodule update --init --recursive
submodule_status="$(git -C "$fast_lio_dir" submodule status --recursive)"
if printf '%s\n' "$submodule_status" | grep -Eq '^[+U-]'; then
  echo 'Error: FAST-LIO submodules do not match their recorded commits.' >&2
  exit 1
fi

# Upstream stores both ROS manifests and ignores package.xml. Colcon needs
# this exact ROS 2 manifest before dependency discovery; build.sh also clears
# the whole workspace, so do not use it for a reproducible incremental build.
livox_driver_dir="$workspace_src/livox_ros_driver2"
if [ -e "$livox_driver_dir/package.xml" ]; then
  if ! cmp -s "$livox_driver_dir/package_ROS2.xml" "$livox_driver_dir/package.xml"; then
    echo 'Error: Livox package.xml differs from its pinned ROS 2 manifest.' >&2
    exit 1
  fi
elif [ -L "$livox_driver_dir/package.xml" ]; then
  echo 'Error: Livox package.xml is a broken symlink.' >&2
  exit 1
else
  ln -s package_ROS2.xml "$livox_driver_dir/package.xml"
fi

for package_name in "${project_packages[@]}"; do
  source_path="$repository_root/ros2_ws/src/$package_name"
  target_path="$workspace_src/$package_name"
  if [ -L "$target_path" ]; then
    if [ "$(readlink -f "$target_path")" != "$(readlink -f "$source_path")" ]; then
      echo "Error: $target_path points to a different source tree." >&2
      exit 1
    fi
  elif [ -e "$target_path" ]; then
    echo "Error: $target_path already exists and is not the expected symlink." >&2
    echo "Move it aside manually after confirming that it contains no needed work." >&2
    exit 1
  else
    ln -s "$source_path" "$target_path"
  fi
done

if git -C "$fast_lio_dir" apply --unidiff-zero --reverse --check \
    "$fast_lio_patch" >/dev/null 2>&1; then
  echo "[setup] FAST-LIO MID-360 patch is already applied."
elif git -C "$fast_lio_dir" apply --unidiff-zero --check "$fast_lio_patch"; then
  git -C "$fast_lio_dir" apply --unidiff-zero "$fast_lio_patch"
  echo "[setup] Applied FAST-LIO MID-360 density/latency/gravity-align patch."
else
  echo "Error: FAST-LIO patch neither applies nor matches the working tree." >&2
  exit 1
fi

fast_lio_status="$(git -C "$fast_lio_dir" status --short)"
unexpected_fast_lio_status="$(printf '%s\n' "$fast_lio_status" |
  grep -Ev '^ M (config/mid360.yaml|src/laserMapping.cpp|src/IMU_Processing.hpp)$' || true)"
if [ -n "$unexpected_fast_lio_status" ]; then
  echo "Error: FAST-LIO contains changes outside the recorded patch:" >&2
  printf '%s\n' "$unexpected_fast_lio_status" >&2
  exit 1
fi
if ! (cd "$fast_lio_dir" &&
      sha256sum -c "$script_dir/patches/fast_lio_mid360.SHA256SUMS"); then
  echo "Error: FAST-LIO files do not match the recorded patched state." >&2
  exit 1
fi

for dependency in Livox-SDK2 livox_ros_driver2 pcd2pgm; do
  dependency_status="$(git -C "$workspace_src/$dependency" status \
    --porcelain --untracked-files=all)"
  if [ -n "$dependency_status" ]; then
    echo "Error: $dependency contains unrecorded local changes." >&2
    printf '%s\n' "$dependency_status" >&2
    exit 1
  fi
done

echo "[setup] Source workspace is reproducible at: $workspace_root"
echo "[setup] Next: install Livox-SDK2, run rosdep, then build as documented in ros2_ws/README.md."
