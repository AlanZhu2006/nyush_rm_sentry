# Shared SP vision application link

The active `SentrySteerNav2Task` owns USB and multiplexes legacy navigation,
SP43 telemetry and SC12 status in one asynchronous transfer. Its static transmit
buffer remains untouched until completion or successful abort; no stack buffer
is passed to the asynchronous USB driver. Input is a bounded byte stream, with
independent nav15 and SP29 CRC checks, arbitrary read fragmentation and recovery
from corrupt candidates. SP commands cross an atomic ISR-to-task mailbox.

## Wire contract

All floats are little-endian IEEE754 binary32. Angles are radians, rates rad/s,
accelerations rad/s², speed m/s. SP signs: yaw right positive, pitch up positive.
The vision client converts its internal signs once at its protocol boundary.

- SP29: `SP`, command mode u8, yaw/yaw_rate/yaw_accel/pitch/pitch_rate/pitch_accel
  f32, CRC u16. Mode 0 releases authority, 1 aims, 2 requests aim and fire.
- SP43: `SP`, requested **work mode** u8 (0 idle, 1 auto aim, 2 small buff,
  3 big buff), quaternion wxyz f32[4], yaw/yaw_rate/pitch/pitch_rate f32,
  bullet speed f32, cumulative bullet count u16, CRC u16. Work mode is independent
  of SP29 command mode: fire command 2 never selects small-buff detection.
- SC12: Python struct `<2sBBBBHHH`: `SC`, version 1, capabilities, flags,
  last accepted command mode, accepted count, rejected count, CRC. Counters wrap.
- Capabilities: YAW=1, PITCH=2, FIRE=4, ATTITUDE=8, BULLET_SPEED=16,
  BULLET_COUNT=32. Flags: ARMED=1, ACTIVE=2, REJECTED=4, TIMEOUT=8,
  MANUAL_OVERRIDE=16, FEEDBACK_FRESH=32.
- SP/SC CRC: init 0xffff, reflected polynomial 0x8408, no final XOR,
  over preceding bytes; stored little-endian. Navigation retains CCITT-FALSE
  init 0xffff, polynomial 0x1021. Golden check `123456789`: SP 0x6f91,
  nav 0x29b1.

Nonfinite fields reject. Absolute angles are bounded to ±3.15, rates ±100,
accelerations ±1000, bullet speed 0..100. Feedback quaternion norm squared must
be 0.95..1.05, and normalized angular disagreement with the Euler-derived
quaternion must be <=0.05 rad. Quaternion and Euler fields describe one calibrated
camera/gimbal sample. The legacy Euler construction is Z(yaw), Y(roll), X(pitch),
with wxyz output; it is not the usual meaning of pitch/roll names.

## Real service configuration and authority

`SentryVisionReadFeedback` is a weak application hook returning unavailable by
default. Override it only with real calibrated feedback in the declared frame,
a sample timestamp from the RTOS millisecond clock, real arm state, requested
work mode, and capabilities backed by both real services and actuators. A raw
body IMU quaternion needs a mounting/joint transform first. No INS, pitch,
bullet or firing feedback is synthesized. Without a provider SP43 stays idle
with zero unavailable attitude and SC capabilities zero.

The original swerve checkout masks all aim/fire capabilities because it only
registers chassis motors. The beyblading checkout can retain YAW when its
existing motor is online and a calibrated provider exists; pitch and fire are
masked because those services are disabled/uninitialized. Bullet capabilities
require actual provider data. A command for an unavailable axis rejects unless
all three fields for that axis are exactly zero; mode 2 rejects without FIRE.
Both controllers therefore currently refuse firing.

Real feedback expires after 100 ms, commands after 200 ms. An explicit provider
work mode 0 revokes cached aim/fire authority and rejects mode-1/2 commands even
with fresh armed feedback; mode-0 release remains accepted. Sensing validity is
independent of this operator request to idle. Invalid commands,
missing feedback/authority and unavailable capabilities revoke vision output.
Corrupt SP candidates also revoke output. Mode 0 releases immediately. Manual
nonzero yaw/pitch input or lower-yaw spin request takes precedence while fresh;
vision can resume only while its command and feedback remain fresh.

SP work mode remains available from a fresh, armed provider with yaw capability
before the first aim command, and after rejected/timed-out commands. This lets
detection start and recover. SC ACTIVE remains clear until an accepted fresh
command applies; REJECTED/TIMEOUT report command failure. Manual override,
unarmed or unavailable feedback produces idle work mode. No error status is
presented as successful actuation.

The beyblading adapter forms an absolute angular error against measured
camera/gimbal attitude and maps that error to measured motor/joint coordinates
using its existing sign calibration. The motor-relative encoder interval is
`[-AIM_YAW_LEFT_TRAVEL_DEG, -AIM_YAW_RIGHT_GUARD_DEG]`; SP right-positive uses
`AIM_YAW_SP_COMMAND_SIGN = -SENTRY_AIM_YAW_COMMAND_SIGN` because the existing
manual convention is left-positive. Targets beyond the existing guarded travel
reject. A bounded position servo plus command derivative feedforward enters the
existing speed/acceleration/braking profile, encoder edge protections and motor
mapping. Absolute angles never enter the manual-rate path as rates.

## Software checks and hardware limits

Run `just vision-protocol-check`, `just vision-bridge-check`, and `just build`.
The first host-compiles the actual application helpers, checks golden bytes,
both CRCs, fragmented/corrupt mixed streams, semantic limits, mode/capability
rejection, timeout/manual override, quaternion consistency and joint mapping;
it also compares helper copies between checkouts. In beyblading, run
`just small-yaw-check` to preserve existing manual-yaw regressions.

No hardware was used. Motor direction, IMU mounting/calibration, real provider,
pitch, firing, actual bullet feedback and live USB timing remain hardware checks.
Never flash as part of these software checks.

The existing non-swerve SP path retains its packet and application behavior,
with finite/range validation added for every command float and one bounded
stream decoder shared by UART and USB receive callbacks. `USARTInstance` has
an additive `recv_len` field containing the actual idle/DMA fragment size,
set immediately before its existing callback. Existing callback signatures
and configured buffer sizes remain compatible. Host tests compile the actual
legacy validation/stream and BSP callback bodies and exercise a 5+24-byte
UART frame, rather than assuming a complete frame in every DMA event.

Legacy telemetry preserves internal multi-turn yaw in degrees, but wraps only
the transmitted yaw phase to ±pi. The quaternion retains the same orientation
(up to its equivalent sign); pitch is not wrapped to conceal invalid feedback.
Host tests compile the actual telemetry builder and Euler helper with 721-degree
internal yaw and mixed pitch/roll, checking one-degree wire yaw and quaternion
coherence without changing internal state.
