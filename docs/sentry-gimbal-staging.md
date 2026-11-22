# Sentry gimbal integration status

This document records the original preview-only development stage. The NUC
deployment procedure and active controls are documented in
[`ros2_ws/README.md`](../ros2_ws/README.md) and
[`application/sentry/SENTRY_STEER_NAV2.md`](../application/sentry/SENTRY_STEER_NAV2.md).

The current `sentry_swerve` firmware runs `SentrySteerNav2Task`, which owns the
USB CDC command stream and the four steering and four drive motors. It does not
initialize the old `GimbalInit` or `RobotCMDInit` applications. The older sentry
configuration's GM6020 yaw/pitch CAN IDs and encoder zeroes describe a different
mechanism and must not be treated as calibration for the new gimbal.
The old `robot_cmd.c` maps the left remote-control rocker to yaw and pitch
angle increments, and mouse X/Y to the same logical axes. Its `GIMBAL_FREE_MODE`
and `GIMBAL_GYRO_MODE` choose feedback sources; they are not large-yaw cruise
or combined pitch/yaw scan modes.

## Axis names

The proposed serial kinematic chain is chassis -> LK9025 large yaw -> upper
platform -> GM6020 small yaw -> head -> GM6020 pitch. "Large" describes the
upstream joint carrying the upper assembly; "small" describes the downstream
joint adjusting the head. These are two distinct relative angles. Confirm the
actual mechanical chain before using this convention in a controller.

## Browser preview

`just wasd` now displays a gimbal preview panel. It publishes
`geometry_msgs/Vector3` on `/gimbal/cmd_vel_preview` at 30 Hz only while the
page owns control and its separate preview toggle is enabled. The gimbal
preview may be enabled without arming the chassis. The fields are
angular-rate requests in rad/s: `x` small yaw, `y` pitch, `z` large yaw.
`J/L` request small-yaw left/right, `I/K` pitch up/down, and `U/O` large-yaw
counterclockwise/clockwise. Simultaneous manual keys are supported. The maximum
preview rates are 0.25, 0.18, and 0.15 rad/s respectively, multiplied by the
page's speed scale.

The cruise preview requests +0.15 rad/s on large yaw. The combined scan preview
requests the same large-yaw rate plus a sinusoidal pitch-rate request at a
six-second period and 0.09 rad/s peak. This is a velocity preview, not a
position-limited scan: motor tracking error could accumulate. It must not be
connected to motor output without pitch feedback and validated limits.

The existing browser watchdog disarms after 350 ms without updates. Switching
mode disarms the gimbal preview until the operator enables it again. Escape,
window blur, page hide, and disconnect release both browser controls; Space
requests zero chassis speed and disarms gimbal preview. The publisher emits a
short zero burst after disarming. No ROS-to-USB bridge subscribes to this preview
topic, and no current MCU task consumes it. Consequently this panel cannot
energize the three gimbal motors.

## Before actuator integration

Record all three actual CAN buses and motor IDs, feedback protocol/firmware
versions, encoder zero references, motor directions, joint gear ratios, pitch
and small-yaw mechanical travel, large-yaw cable/slip-ring travel, and which
rigid body carries each IMU and MID-360. The old sentry pitch min/max are both
zero and are not valid travel limits. The repository's LK9025 driver uses a
16-bit angle scale whereas its included protocol example describes a 14-bit
encoder; verify the exact hardware version and repair/test that driver before
using it for the large-yaw actuator.

The actuator stage needs its own MCU-side gimbal state machine, feedback
freshness checks for all participating axes, calibrated software limits and
stop paths, measured joint-state telemetry, a separate browser-to-ROS-to-USB
command type, and a lifted/isolated single-axis test. Cruise and combined scan
must be validated against cable travel before continuous operation.
