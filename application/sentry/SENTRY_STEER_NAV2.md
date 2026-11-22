# Sentry Nav2 swerve control

This standalone control path reuses the steering zero positions, motor PID
settings, CAN assignments and wheel geometry from the
keyboard steering test. It replaces eight discrete directions with continuous
body-frame velocity commands.

## Data flow

1. `scripts/sentry_nav2_bridge.py` subscribes to the final ROS 2 `/cmd_vel`.
2. The standalone browser launch and generic bridge limit translation to
   0.80 m/s and yaw rate to 2.4 rad/s. Autonomous navigation retains its
   separate configured limits.
3. A CRC-protected USB CDC frame carries `vx`, `vy`, and `wz` to the controller.
4. `SentrySteerNav2Task()` computes one velocity vector per swerve module.
5. Each steering target is optimized by reversing its drive wheel when that
   avoids more than 90 degrees of steering motion.
6. Motor telemetry returns over USB and the bridge publishes `/wheel/odom`.

ROS coordinates are `+x` forward, `+y` left, and `+yaw` counter-clockwise.
Controller command units are mm/s and mrad/s. Motor order is LF, RF, LR, RR.

## Browser aiming and lower-yaw request

Run `just wasd` and enable `ENABLE SMALL YAW` separately from chassis control.
Hold `J` or the left button for a positive logical small-yaw rate, and `L` or the right
button for a negative logical rate. The MCU maps positive logical input to
decreasing motor encoder angle, matching physical left on this mechanism.
The maximum request is 0.60 rad/s (34.4 degrees/s),
scaled by the selected speed setting. Releasing the key sends zero rate;
inside the permitted range it holds the target, while at an edge it stops
output. Escape, Space, losing page focus, and the
350 ms browser watchdog release small-yaw control. A yaw-only session does
not publish chassis commands, including when it exits.
The 20/40/60/80/100 percent settings scale translation, Q/E and J/L. Chassis
maximums remain 0.8 m/s and 2.4 rad/s; B-mode chassis and lower-yaw requests
remain 0.5 rad/s. MCU chassis caps are 800 mm/s, 2400 mrad/s and 2500 RPM.
Only small yaw has the increased 0.60 rad/s request; its USB clamp and target
integrator both accept up to 600 mrad/s. The bridge's pitch limit is unchanged.

The browser sends these rates in the `x` field of `/sentry/gimbal_manual`;
the USB bridge encodes them in a second 15-byte CRC-protected frame (type
`0x02`). The current configuration assigns upper GM6020 yaw to CAN2 motor
ID 4 (feedback ID `0x208`); this setting must match the physical wiring.
The inherited ID 6 belongs to the lower yaw. On 2026-10-28, the operator placed
small yaw at its right mechanical stop; read-only feedback was stable at ECD
7309. `SENTRY_AIM_YAW_RIGHT_LIMIT_ECD` stores that absolute position, so a
reboot does not redefine the stop. Encoder angle increases toward physical
right. The mechanism relaxed from 7309 to about 7303 after reset, so the
controller stops rightward requests sixteen encoder counts (0.703 degrees)
before the recorded stop. At or beyond this inward guard, rightward
and zero requests stop the motor and clear active PID history; leftward input
can leave the boundary in small increments without jumping to a new reference.
A motor-task output hook also clips any positive residual PI output at the
edge. This protection is independent of the browser and ROS command source.
On 2026-10-30, the operator confirmed the current position as the left stop;
read-only feedback remained at ECD 4569. `SENTRY_AIM_YAW_LEFT_LIMIT_ECD`
stores this second absolute boundary. The physical span is 2740 counts
(120.410 degrees). A sixteen-count inward guard also applies at the left:
the permitted target interval is ECD 4585 through 7293 (119.004 degrees),
with no encoder wrap on this calibration. At or below ECD 4585, leftward
and zero requests stop output and clear active PID history; rightward input
can leave gradually. The output hook independently blocks negative residual
PI output there. Both stops remain fixed after controller or motor power cycles.
Observed tracking/release at ECD 4590 left the motor five counts inside the
left soft boundary while the former exact-threshold status still said clear.
Limit state now engages within eight counts (0.352 degrees) of a soft edge
and stays latched until moving thirty-two counts (1.406 degrees) inside it.
For this calibration, left engages at ECD <=4593 and releases at >=4617;
right engages at >=7285 and releases at <=7261. The same latched flags block
outward/zero control and outward motor output, in addition to the original
exact target/output clamps. An inward request remains allowed. A restart
at ECD 4590 therefore immediately reports the left software limit. This is
encoder-based software protection; there are no physical limit-switch inputs.
The MCU stops output when aim commands expire after 300 ms. Chassis/B mode
continues publishing the shared aim frame, so zero yaw rate can still hold
the current angle while that mode remains enabled. The chassis command
remains type `0x01`, so aiming cannot silently change chassis `wz`.

USB telemetry status bit `0x40` identifies firmware that reports small-yaw
feedback, and bit `0x04` reports a fresh GM6020 feedback stream. The bridge
publishes `/sentry/aim_yaw_online` (`std_msgs/Bool`) for each valid supported
telemetry frame. The browser requires a fresh online sample (within 250 ms)
to arm small-yaw control. Missing capability, missing USB telemetry, or stale
ROS feedback displays unknown instead of claiming the motor is offline.
Offline or expired feedback disarms manual yaw and requires enabling it again.
The original type `0x81` frame retains status bit `0x80` for the right guard.
Both directional permissions now require a dedicated type `0x82` frame, so
older firmware cannot silently permit motion without both calibrated limits.
The new frame is also 29 bytes including CRC16 and uses the little-endian
Python body format `<2sBBHIHfffhB`: magic, version, type, sequence, MCU time
in ms, encoder, actual total angle in degrees, target total angle in degrees,
speed in degrees/s, signed control output, and flags. Flags are online `0x01`,
left guard `0x02`, right guard `0x04`, and active angle controller `0x08`.
The high bits add chassis readiness: reporting supported `0x10`, all steering
feedback online `0x20`, all drive feedback online `0x40`, and controller output
enabled `0x80`. High status bits without their capability bit are rejected.
Older low-four-bit frames remain valid yaw feedback but report chassis state
as unknown. The HTTP state exposes `chassis_ready`, steering/drive group
online state, actual output enable and `chassis_block_reason`; the page shows
why the base cannot move. The chassis and small-yaw arm buttons stay separate.
The two frames share one 58-byte USB transfer every 50 ms; chassis cadence
and the old frame layout are unchanged. The bridge publishes
`/sentry/aim_yaw_left_limit` and `/sentry/aim_yaw_right_limit` as `std_msgs/Bool`.
The page and HTTP server suppress `J` at the left guard and `L` at the right;
unknown or expired feedback blocks its corresponding direction. The other
direction remains available to leave a boundary. Firmware enforces the same
restriction for direct ROS or USB input. Tests exercise the compiled C helper
without issuing commands into either mechanical stop.

`/sentry/aim_yaw_feedback` is `std_msgs/Float64MultiArray` containing
`[ecd, actual_deg, target_deg, speed_deg_s, output, flags, mcu_ms]`.
The HTTP state endpoint exposes these diagnostics and tracking error, with
stale numeric values returned as `null`; the page displays tracking error.
Output is the clipped motor-task command (zero while inactive), not amperes.

The standalone yaw speed PI uses robot-specific Kp 20 and Ki 40, a 6000
output cap, and a 3000 integral cap. Its initialization explicitly enables
`PID_Integral_Limit`; without this flag the PID implementation ignores the
configured integral cap. Position P remains 8. The position deadband
is now explicitly 0.10 degrees. A motor-frame reference profile uses 2 rad/s^2
acceleration and 3 rad/s^2 braking; it slows near a soft boundary according to
`sqrt(2 * braking * remaining_distance)`, considering the closer of actual and
target distances. Its speed envelope also applies after the acceleration ramp.
These are reference-profile parameters, not measured physical braking limits;
the absolute output hook and latched limit interlocks remain independent.

The existing DJI `SPEED_FEEDFORWARD` interface receives 0.8 times the actual
limited target increment divided by dt, bounded to the configured target-rate
range before applying this gain. It adds to the position-loop correction before
the speed PI. Target, feedforward and motor enable are published atomically.
Offline feedback, expired/disabled commands, invalid profile input or outward
limit requests clear both feedforward and profile state. Ordinary key release
immediately zeroes profile/feedforward and retains position hold. Pitch does not
inherit the yaw feedforward pointer. Position correction remains separately
capped at 120 deg/s; 0.60 rad/s is the operator target rate, not a hard cap on
every transient motor speed.

The recorded pre-change 0.30 rad/s test has continuous 100 Hz feedback. After
omitting each ramp's first second, right/left motion averaged 17.06/17.85 deg/s
with mean position lag 1.56/2.24 degrees. Maximum reversal error was 5.86 degrees;
reverse speed reached the request after about 0.49 s. No steady 100 ms
angle-difference window was below 2 deg/s. With position P8 and no feedforward,
nominal tracking needs about 17.19/8=2.15 degrees of error, consistent with
this lag. This supports feedforward and reference shaping without declaring
new optimal P/I values. Updated physical-motion logs are needed for comparison.
The baseline plot and report are in `monitor/yaw_pid_analysis_20261030/`.
The subsequent 0.60 rad/s test showed actual angle oscillation: approximately
1.37 degrees peak-to-peak with a 19.8 Hz sampled spectral peak. Interior
snapshots of the former PI50/200 frequently implied a proportional term above
9000 raw units, enough to reach the 6000 output cap despite a 3000 integral
cap. These are snapshot inferences, not a measurement of every PID cycle.
Kp and Ki were therefore reduced to 20/40 while retaining the integral/output
caps, feedforward and rate profile. This is a conservative tuning candidate;
fresh movement is required to establish whether the oscillation is resolved.
The recorded leftmost point was ECD 4727, with target about 4690, both above
the left-entry threshold 4593. At that time the reference stopped advancing;
the old RTT packet does not include input keys or limit/enable history, so it
cannot identify a mechanical obstruction or explain why the input stopped.
Use `capture_small_yaw_state.py` alongside RTT for subsequent tests: its
read-only HTTP samples record keys, enable state, output, limits and feedback
age. The 20 Hz HTTP record classifies sustained stops; use 100 Hz RTT angle
data for the faster oscillation. Reports are in `monitor/yaw_vibration_analysis_20261030/`.
The present motor-feedback speed path converts integer RPM into degrees/s,
so its raw resolution is 6 degrees/s. This is coarse against the browser's
34.38 degrees/s maximum and 6.88 degrees/s lowest setting. Speed quantization,
static friction, and integral accumulation are plausible sources of low-speed
stick-slip; the command rate itself does not prove their cause. The previous
reference had no angular-acceleration ramp, which made starting
and reversing abrupt.
For diagnosis, compare a short interior movement's continuous angle target
against actual angle, speed, output, active state and feedback ages before
changing PID gains, filters, or feedforward. No extra physical movement was
used to set either boundary or to validate this code update.

`just logger --backend jlink` exposes the RTT viewer on port 8765. The standalone
swerve snapshot now fills the existing dashboard yaw target/actual angle,
target/actual speed and raw encoder fields at 100 Hz; the yaw online bit also
reflects the standalone CAN2 ID4 feedback. Dashboard v8 stays 415 payload
bytes and 425 frame bytes. Earlier exports with placeholder zero gimbal data
cannot reconstruct small-yaw motion performed before this mapping was added.
Read a monitor export without claiming the probe using
`just _py scripts/diagnostics/logger_feedback_health.py monitor/<export>.txt`.
It summarizes recent frame cadence, sequence gaps, online bitmaps and available
yaw motion. Use
the USB `0x82` telemetry or the verified read-only probe diagnostic to check
small yaw independently.

Aim feedback validity uses a received eight-byte CAN frame and the existing
motor watchdog. It must not use the DJI driver's `feed_cnt` as a packet count:
that field stores a DWT cycle timestamp. H7 DWT initialization unlocks the
Cortex-M7 trace registers and restores cycle counting after debugger detach.
Application command deadlines and manual target integration use the RTOS tick clock.
USB suspend/resume restores the PHY clock when low-power mode is disabled.

Read-only hardware inspection is available through
`just _py scripts/diagnostics/small_yaw_status.py`. The tool verifies allocated
flash sections against the ELF before inspecting motor pointers; unwritten
linker padding is excluded. `check_small_yaw_live.py` reads the browser status
by default. Its explicit `--jog` option makes only one 20%-speed request for
up to 0.1 seconds (roughly 0.17 degrees of target change) and then releases
control. This is a small command-path check, not mechanical-limit calibration
or proof that the motor overcame static friction. Do not repeat or enlarge it
to infer travel limits. At the doubled yaw speed the duration shortens to
about 0.025 seconds to preserve the same small target change.

The inherited sentry config says pitch is CAN2 ID 7 and sets both angle limits
to zero. A separate `controls_typec` sentry config says CAN2 ID 5 with limits
-28 to +25 degrees. Neither identifies the live motor in this robot. Pitch is
disabled by `SENTRY_AIM_PITCH_ENABLED=0` until its ID, polarity and physical
limits are verified on the robot. The browser suppresses up/down input. Keep
the mechanism unpowered while identifying the motor and measuring its free
travel; leave margin inside both hard stops when setting software limits.

The operator confirmed the lower yaw is DaMiao on CAN1 ID 6. The speed-mode
backend uses the existing bench-profile master ID 0 and is enabled by
`SENTRY_LOWER_YAW_DM_ENABLED=1`. B toggles its 0.5 rad/s request; toggling it
off commands zero speed, and a stale or disabled browser command stops the motor.
The type `0x02` frame carries the lower-yaw request separately from chassis
`wz`. The browser's B mode currently also requests a fixed 0.5 rad/s chassis
spin, with W/A/S/D translation still available. Verify direction and actual
speed with the robot supported before increasing the configured rate.

## Safety behavior

- Commands expire after 300 ms.
- A missing steering or drive motor stops every chassis motor.
  On 2026-10-30 the RTT export consistently showed steering `0xF` and drive
  `0xB`: the left-rear M3508, CAN1 ID4 (feedback `0x204`), was missing. This
  explains WASD output being disabled even with healthy USB and RTT streams;
  the all-eight-online interlock remains in effect. A GM6020 ID4 feedback
  frame is `0x208`, so matching numeric motor IDs do not replace this feedback.
- A fresh, enabled zero-speed command keeps the previous steering direction.
  After motion, the drive motors actively brake until all four remain below
  100 RPM for 30 ms, with a 200 ms hard timeout, and then stop their closed
  loops. Enabling control while already stationary leaves the drive motors
  stopped, preventing zero-speed feedback noise from exciting a lifted wheel.
  A disabled, stale, or offline command still hard-stops every motor.
- PID history is cleared whenever control re-enters from the disabled state, so
  the previous command cannot inject stale integral or filtered output into the
  next movement.
- All active drive modules share one steering-alignment scale. Drive remains
  zero while any active module is more than 16.875 degrees from its target,
  then all four requested RPMs ramp together and reach full output at 5.625
  degrees. Preserving their ratios prevents asymmetric wheel force from
  changing the requested chassis direction during steering transitions.
- A module with negligible requested speed holds its last meaningful steering
  target to avoid jitter around an undefined zero-speed direction. On first
  use it adopts the current angle; a later zero-speed crossing therefore does
  not interrupt and restart an active turn.
- Nav2 steering uses the proven proportional-only outer loop (`Kp=20`) and
  filtered proportional speed loop (`Kp=40`) from the original keyboard
  controller. A 720 deg/s outer limit keeps direction changes fast, while the
  7000-command current cap prevents the measured +/-15000 limit cycle.
- The drive-speed loop is an unfiltered proportional-only loop (`Kp=6`) with a
  120 RPM error deadband. This avoids chasing feedback noise and removes the
  measured phase-lag limit cycle. Integral action should only be reintroduced
  after repeatable loaded tests show a steady RPM bias without oscillation.
- RTT diagnostics report each steering error and target/actual logical drive
  RPM every 500 ms for zero-offset and closed-loop verification.
- Dashboard protocol v8 publishes the four steering target/actual angles,
  steering speed-loop references and feedback, controller output/current,
  drive RPM, and drive output/current at 100 Hz. The browser `Swerve` tab keeps
  continuous plots and the monitor export stores the same samples in TSV form.
- Closing the bridge sends three disabled zero-speed frames.
- USB CDC output is copied into a persistent transmit buffer before the
  asynchronous transfer starts. A transfer that remains busy for 250 ms is
  aborted and retried, so one lost IN completion cannot permanently stop wheel
  odometry.
- The ROS bridge closes and rediscovers the NYUSH USB controller when telemetry
  remains stale for 1.5 seconds. Reconnection resets only the MCU timestamp
  baseline; the accumulated odometry pose is retained.

## Build and run

Build the firmware without flashing:

```sh
just build sentry_swerve
```

After explicitly flashing the firmware, run the bridge on the NUC:

```sh
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 run mid360_nav_bringup sentry_nav2_bridge.py
```

Automatic selection matches USB VID/PID `0483:00ca`, so it remains valid if
Linux changes `ttyACM0` to another number after re-enumeration. A fixed `port`
parameter should only be used when more than one matching controller is
connected. Stale warnings include received-byte, valid-frame, CRC-error, and
discarded-byte counters for distinguishing a silent controller from malformed
frames.

Keep the chassis lifted for the first sign and steering-direction checks.

## RTT swerve plots

Stop any existing RTT reader before starting the browser bridge because one
J-Link probe can only be owned by one reader:

```sh
just logger
```

Open `http://JETSON_IP:8765/` from a device on the same LAN and select
`Swerve`. A stable target
with oscillating angle error indicates closed-loop tuning or mechanical
backlash; a jumping target points to direction selection or command input; a
stable steering trace with oscillating drive RPM isolates the drive loop.
