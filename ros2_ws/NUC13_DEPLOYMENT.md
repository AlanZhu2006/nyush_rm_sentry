# NUC13 deployment supplement

Validated on 2026-09-07 on `nyush-NUC13ANKi5`: Ubuntu 22.04.5, i5-1340P,
16 GB RAM, ROS 2 Humble, x86_64. Source baseline: branch
`omni-full-stack-repro`, commit `44644305d0c3d3f8dd4624ffd9f62f67f7c9e760`,
with the local deployment fixes visible in the working tree.

This supplements, not replaces, the original Jetson hardware test record.
Software deployment and offline runtime checks passed. No controller, debug
probe, or MID-360 was connected; no firmware was flashed and no motion or
real localization was tested. The old NUC12 installation was not available
for direct comparison.

## Installed layout

- Firmware and deployment scripts: `/home/nyush/work/nyush_rm_sentry`
- Pinned ROS source/build/install workspace: `/home/nyush/robot_ws`
- Both project ROS packages in `robot_ws/src` link to the original
  `nyush_rm_sentry` checkout after the NUC branch consolidation.
- Tools: `~/.local/bin`, firmware/debugger Python environment: repository `.venv`
- Environment: `~/.config/nyush-sentry/env.bash`; load with `sentry-env` in Bash
- SDK library: `~/.local/lib`; J-Link: `~/.local/opt/JLink_Linux_V972_x86_64`
- Robot configuration: local `.robot_type` selects `sentry_swerve`

The host package/configuration entry point is `just finish-nuc-deploy`. Run it
in the user's terminal for sudo authentication. It does not flash or start a
live robot stack. It requires the tools/source checkout already installed on
this NUC; it is not a bare-OS bootstrap for arbitrary hosts. Package changes
and previous network state are recorded under `/var/backups/nyush-sentry-host.*`.

All six workspace packages built successfully: `icp_probe`,
`livox_ros_driver2`, `livox_sdk2`, `pcd2pgm`, `fast_lio`, `mid360_nav_bringup`.
The complete colcon log is `~/robot_ws/log/build_2026-09-07_13-04-37`.
`just ros-deps-check` reports all system dependencies satisfied.

## This host differs from the Jetson defaults

| Setting | NUC13 value |
| --- | --- |
| LiDAR NIC/profile | `enp86s0` / `nyush-mid360` |
| NUC / MID-360 IPv4 | `192.168.1.50/24` / `192.168.1.182` |
| Ethernet default route | Disabled; Wi-Fi is retained |
| Controller alias after connection | `/dev/sentry-controller` |
| Serial/debug permissions | `dialout`, `plugdev`; log in again after installation |
| Normal ROS domain / RMW | `42` / `rmw_cyclonedds_cpp` |
| VNC service | `systemctl --user status nyush-sentry-vnc.service` |
| VNC display/listener | `:2`, localhost-only TCP 5902; GNOME `:1` is untouched |
| VNC Xauthority | `~/.vnc/sentry-Xauthority`, mode 0600 |

VNC is enabled with user lingering and does not depend on an active GDM login.
Connect through `ssh -N -L 5902:127.0.0.1:5902 nyush@NUC_IP`, then point the
VNC client to `127.0.0.1:5902`. The generated password is stored privately in
`~/.config/nyush-sentry/vnc-password`; do not commit or share it. `just vnc-setup`
retains the password and restarts the desktop only if its installed unit or
startup script changed.

Always load `sentry-env` before `just map` / `just nav`: the original launchers
otherwise assume the Jetson's VNC display `:1` and a different authority file.
For a pre-existing shell, explicitly source `~/.config/nyush-sentry/env.bash`.

## Verification commands and limits

```bash
just repro-check
just ros-deps-check
just build sentry_swerve 8
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp just offline-check
just runtime-check
just deployment-check
```

`offline-check` uses a PTY in localhost domain 181 to exercise the real bridge
and odometry adapter: independent CRC checks, velocity scaling, timeout stop,
telemetry parsing/recovery, TF offsets, and three disabled shutdown frames.

`runtime-check` uses localhost domain 182 and synthetic TF/odometry/scans.
It resolves both installed launch descriptions, loads Livox message support,
checks native shared libraries, activates the seven Nav2 lifecycle nodes plus
the map server, verifies Omni MPPI and map/costmap output, loads the ICP PCD,
and opens RViz on VNC. It never starts Livox or the USB bridge and never sends
a navigation goal. Its temporary DDS profile raises the unicast participant
index limit to 64; it does not change the normal domain 42 network settings.
It closes its own processes and retains a log directory plus a desktop PNG.

The successful runtime evidence is retained locally under
`.cache/deployment/runtime-check-20260907-1321/`. The screenshot shows the
recorded map, costmap, TF and an active Navigation panel. RViz logged a GLSL
sampler warning on initial map display; the map and costmap did render, so
this is not a claim of zero runtime warnings. This test intentionally has no
AMCL and no real ICP alignment; the generic panel's Localization indicator
is not an acceptance test for this ICP-based stack.

## Hardware handoff

Open a fresh SSH login, connect the powered MID-360 to Ethernet and the H723
controller to USB, and confirm the old robot calibration is applicable.
Recheck sensor addressing/extrinsics, map provenance, wheel zero encoders,
motor directions and emergency stop before any powered motion test.

```bash
sentry-env
cd /home/nyush/work/nyush_rm_sentry
just nav arena
```

Start `just wasd --host 127.0.0.1` from a second prepared terminal only when
manual control is intended; it reuses the navigation bridge. Use a different
new map name for `just map` (Ctrl-C auto-saves) to preserve the recorded arena.
Do not run mapping and navigation together. No live stack is left running by
the deployment verification.
