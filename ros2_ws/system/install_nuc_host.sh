#!/usr/bin/env bash
# Install the Ubuntu 22.04 host dependencies for the recorded swerve stack.
# Run as: sudo bash ros2_ws/system/install_nuc_host.sh nyush enp86s0
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
packages=(
  build-essential cmake git pkg-config libapr1-dev libaprutil1-dev libeigen3-dev libpcl-dev
  libusb-1.0-0-dev libhidapi-hidraw0
  gcc-arm-none-eabi libnewlib-arm-none-eabi dfu-util openocd gdb-multiarch
  python3-pip python3-venv python3-colcon-common-extensions python3-rosdep
  python3-vcstool python3-serial
  ros-humble-navigation2 ros-humble-nav2-bringup
  ros-humble-nav2-mppi-controller ros-humble-nav2-smac-planner
  ros-humble-pcl-conversions ros-humble-pcl-ros ros-humble-pointcloud-to-laserscan
  ros-humble-rmw-cyclonedds-cpp ros-humble-robot-localization
  ros-humble-slam-toolbox ros-humble-twist-mux ros-humble-xacro
  ros-humble-ament-cmake-clang-format ros-humble-ament-cmake-clang-tidy
  ros-humble-ament-cmake-black
  tigervnc-standalone-server tigervnc-common tigervnc-tools xfce4 dbus-x11
)

if [[ "${1:-}" == --simulate ]]; then
  exec apt-get --simulate --no-install-recommends install "${packages[@]}"
fi

if [[ "$EUID" != 0 || "$#" != 2 ]]; then
  echo 'Usage: sudo bash ros2_ws/system/install_nuc_host.sh LOGIN_USER LIDAR_NIC' >&2
  exit 2
fi
login_user="$1"
lidar_nic="$2"
id "$login_user" >/dev/null
[[ "$login_user" != root && "$login_user" =~ ^[a-z_][a-z0-9_-]*$ ]]
[[ "$lidar_nic" =~ ^[A-Za-z0-9_.:-]+$ && -d "/sys/class/net/$lidar_nic" ]]
[[ ! -d "/sys/class/net/$lidar_nic/wireless" ]]
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]]
[[ -r /opt/ros/humble/setup.bash ]]

backup_dir="$(mktemp -d /var/backups/nyush-sentry-host.XXXXXX)"
dpkg-query -W > "$backup_dir/packages-before.txt"
nmcli connection show > "$backup_dir/network-before.txt"
ip route show > "$backup_dir/routes-before.txt"

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get --no-remove --no-install-recommends install -y "${packages[@]}"
usermod -aG dialout,plugdev "$login_user"
loginctl enable-linger "$login_user"

rules_target=/etc/udev/rules.d/70-nyush-sentry.rules
if [[ -e "$rules_target" ]]; then
  cp -a "$rules_target" "$backup_dir/"
fi
install -m 0644 "$script_dir/70-nyush-sentry.rules" "$rules_target"
udevadm control --reload-rules

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi

# A separate profile leaves existing Ethernet and Wi-Fi profiles recoverable.
connection_name=nyush-mid360
if nmcli -g NAME connection show | grep -Fxq "$connection_name"; then
  [[ "$(nmcli -g connection.interface-name connection show "$connection_name")" == "$lidar_nic" ]]
  [[ "$(nmcli -g ipv4.addresses connection show "$connection_name")" == 192.168.1.50/24 ]]
  [[ "$(nmcli -g ipv4.never-default connection show "$connection_name")" == yes ]]
else
  nmcli connection add type ethernet con-name "$connection_name" ifname "$lidar_nic" \
    connection.autoconnect yes connection.autoconnect-priority 100 \
    ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.never-default yes \
    ipv6.method disabled
fi

dpkg-query -W > "$backup_dir/packages-after.txt"
echo "Host dependencies installed. Backup: $backup_dir"
echo "Open a new login session for dialout membership. Connect MID-360 to $lidar_nic."
echo 'VNC is installed; set its password and configure its display separately.'
