# Foxglove robot operator

The NUC runs Foxglove Bridge and a workflow supervisor. The **NYUSH Sentry 操作台** panel uses them to start sessions, manage maps, reserve manual chassis control, and view RTT text. Native Foxglove panels display the ROS data.

## Install on the NUC

Set up the [ROS 2 Humble robot workspace](../README.md). The NUC also needs Bash, Just, and `python3-venv`. Install `ros-humble-foxglove-bridge` or set `FOXGLOVE_BRIDGE_PREFIX` to the ROS prefix that contains `lib/foxglove_bridge/foxglove_bridge`. The operator uses Python `websockets` 15 or newer. From the project checkout, prepare the project's default virtual environment.

```bash
just py-bootstrap
.venv/bin/python -m pip install 'websockets>=15'
```

For a different Python environment, set `FOXGLOVE_PYTHON` to its interpreter path. From the checkout, run `just foxglove` for a foreground session. Set `ROBOT_WS` if the built workspace is outside `~/robot_ws`.

For a persistent user service, run `mkdir -p "$HOME/.config/nyush-foxglove"` and create `~/.config/nyush-foxglove/environment` with the absolute paths for your installation.

```ini
SENTRY_PROJECT=/absolute/path/to/nyush_rm_sentry
ROBOT_WS=/absolute/path/to/robot_ws
```

Add `FOXGLOVE_PYTHON` or `FOXGLOVE_BRIDGE_PREFIX` to that file if needed. From the project checkout, install and start the unit.

```bash
install -Dm644 ros2_ws/foxglove/nyush-foxglove.service \
  "$HOME/.config/systemd/user/nyush-foxglove.service"
systemctl --user daemon-reload
systemctl --user enable --now nyush-foxglove.service
systemctl --user status nyush-foxglove.service
```

The service starts in idle mode. To start the user manager at boot before login, enable lingering for the service account with `loginctl enable-linger "$USER"`.

## Connect

Open the built `.foxe` file in Foxglove Desktop and add **NYUSH Sentry 操作台** and a **3D** panel to a layout. Foxglove requires a developer seat for local extensions.

Set up an SSH alias named `sentry-nuc` on the operator computer. Replace the example hostname and user with the ones for your installation in `~/.ssh/config`.

```sshconfig
Host sentry-nuc
  HostName nuc.example.net
  User robot
```

If the extension archive is staged in the NUC user's data directory, copy it to the operator computer.

```bash
scp sentry-nuc:.local/share/sentry-foxglove/nyushrobotics.sentry-operator-0.1.0.foxe .
```

Open the SSH tunnel on the operator computer and keep it running while connected.

```bash
ssh -N -L 127.0.0.1:8766:127.0.0.1:8766 sentry-nuc
```

In Foxglove choose **Foxglove WebSocket** and connect to `ws://127.0.0.1:8766`.

## Layout and operation

| Task | Controls and data |
| --- | --- |
| Mapping | Enter a new map name, select Start mapping, and wait for a live dual-LiDAR cloud. In a 3D panel set the display frame to `camera_init`, then enable `/lidar/points_fused` and `/scan`. Select Save map to snapshot or Save and stop to close the session. |
| Manual control | Reserve control in the operator panel. Hold the direction buttons or Space with WASD and Q/E in the focused control area. The NUC reuses the existing WASD control lease and 350 ms command watchdog. Releasing control sends a stop. Manual reservations also expire after the panel stops sending its heartbeat. |
| Navigation | Select a saved map. In the navigation 3D panel use the `map` frame, an overhead view, `/map`, `/scan`, and `/plan`. A map is ready once ICP is localized and Nav2 is accepting goals. The controller telemetry indicator must also be live to send a goal. |
| Initial pose and goals | In the 3D panel's **Publish** settings, set **2D pose estimate** to `/sentry/operator/initialpose` and **2D pose** to `/sentry/operator/goal`. Use the estimate if ICP needs an initial pose. Selecting a goal stages it in the operator panel. Confirm it with Send goal. Cancel goal cancels the owned Nav2 goal. |
| RTT | Select Start RTT to attach the existing logger in its normal connection mode. Text appears in the operator panel. A missing or busy probe appears in the log. Add Plot series for `/sentry/operator/rtt/chassis_power_w.data`, `chassis_voltage_v.data`, `wheel_rpm.data[0]` and `wheel_target_rpm.data[0]` under the same prefix. Wheel indices 0–3 are left front, right front, left rear and right rear. |

Mapping and navigation share the existing workspace lock. A terminal-owned session keeps that lock and must be stopped in its own terminal. The operator never takes over its process. A failed save keeps mapping running so it can be retried. Stop without saving is a separate confirmed action. End an active mapping session with the panel's Save and stop before stopping the user service.

## Backend and boundaries

The operator exposes a fixed command topic and returns a receipt with each request ID. The mode and readiness come from live ROS observations and child process state. The stock bridge is read-only on loopback port `18766`. The operator endpoint on `8766` forwards visualization and publishes only the four typed command, drive and pose topics. Shell arguments are generated from fixed recipes and validated map names. The stack uses local-only ROS discovery and runs the existing `just map`, `just map-save`, `just nav`, `just wasd` and `just logger` workflows. The WASD HTTP server and RTT socket bind only to `127.0.0.1` while managed by this operator.

Use the operator endpoint from a trusted workstation and allow SSH access only to trusted operators. Access control depends on the workstation and SSH account. Keep both ends of the tunnel on loopback and close it when finished.

The underlying Nav2 guard, chassis limits, mux and watchdog remain responsible for motion gating. The panel's stop and cancel buttons are software commands. The logger adapter reads both streams from one local connection. It uses the existing firmware frame parser and publishes a selection of chassis measurements at up to 20 Hz for the Plot panel. The full RTT monitor capture remains available on the NUC.

## Rebuild the panel

The panel was built with Node.js 24 and npm 11. From `ros2_ws/foxglove/panel`, install the locked dependencies, check the TypeScript, and build the archive.

```bash
npm ci
npm run check
npm run package
```

The `.foxe` file is written to the panel directory. It contains the compiled extension, the project license, and third-party notices.
