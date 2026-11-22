#!/usr/bin/env python3
"""Bridge ROS 2 body velocity commands and the sentry swerve controller over USB CDC."""

import math
from pathlib import Path
import sys
import struct
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
import serial
from serial.tools import list_ports
sys.path.insert(0, str(Path(__file__).resolve().parent))
from sentry_vision_transport import SharedVisionTransport, RELEASE
from std_msgs.msg import Bool, Float64MultiArray


MAGIC = b"\xa5\x5a"
PROTOCOL_VERSION = 1
COMMAND_TYPE = 0x01
GIMBAL_COMMAND_TYPE = 0x02
TELEMETRY_TYPE = 0x81
AIM_TELEMETRY_TYPE = 0x82
COMMAND_ENABLE = 0x01
STATUS_AIM_YAW_ONLINE = 0x04
STATUS_AIM_YAW_FEEDBACK_SUPPORTED = 0x40
STATUS_AIM_YAW_RIGHT_LIMIT = 0x80
COMMAND_BODY = struct.Struct("<2sBBHhhhB")
TELEMETRY_BODY = struct.Struct("<2sBBHI4H4hB")
AIM_TELEMETRY_BODY = struct.Struct("<2sBBHIHfffhB")
CRC = struct.Struct("<H")
COMMAND_FRAME_SIZE = COMMAND_BODY.size + CRC.size
TELEMETRY_FRAME_SIZE = TELEMETRY_BODY.size + CRC.size

MOTOR_COUNT = 4
ECD_FULL_RANGE = 8192
WHEEL_RADIUS_M = 0.060
WHEEL_REDUCTION_RATIO = 19.0
WHEEL_BASE_M = 0.350
TRACK_WIDTH_M = 0.300
STEER_ZERO_ECD = (6141, 6814, 4081, 1358)
STEER_DIRECTION_SIGN = (1, 1, 1, 1)
DRIVE_FEEDBACK_SIGN = (-1, 1, -1, 1)
MODULE_X_M = (
    0.5 * WHEEL_BASE_M,
    0.5 * WHEEL_BASE_M,
    -0.5 * WHEEL_BASE_M,
    -0.5 * WHEEL_BASE_M,
)
MODULE_Y_M = (
    0.5 * TRACK_WIDTH_M,
    -0.5 * TRACK_WIDTH_M,
    0.5 * TRACK_WIDTH_M,
    -0.5 * TRACK_WIDTH_M,
)


def crc16_ccitt(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE for the firmware wire protocol."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def choose_port(requested: str) -> str:
    if requested:
        return requested

    nyush_ports = [
        port.device
        for port in list_ports.comports()
        if port.vid == 0x0483 and port.pid == 0x00CA
    ]
    if len(nyush_ports) == 1:
        return nyush_ports[0]

    acm_ports = [port.device for port in list_ports.comports() if "ttyACM" in port.device]
    if len(acm_ports) == 1:
        return acm_ports[0]
    if not acm_ports:
        raise RuntimeError("No USB CDC controller found; set the 'port' parameter.")
    raise RuntimeError(
        f"Multiple USB CDC devices found: {', '.join(acm_ports)}; set the 'port' parameter."
    )


class SentryNav2Bridge(Node):
    """Translate continuous ROS body twists and swerve telemetry."""

    def __init__(self) -> None:
        super().__init__("sentry_nav2_bridge")
        self.declare_parameter("port", "")
        self.declare_parameter("vision_pty", "/tmp/nyush-rm-sentry-vision")
        self.declare_parameter("command_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("serial_reconnect_delay_s", 0.5)
        self.declare_parameter("telemetry_reconnect_timeout_s", 1.5)
        self.declare_parameter("max_translation_m_s", 0.80)
        self.declare_parameter("max_angular_rad_s", 2.40)

        self._requested_port = str(self.get_parameter("port").value)
        command_topic = str(self.get_parameter("command_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        if command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        reconnect_delay = float(self.get_parameter("serial_reconnect_delay_s").value)
        reconnect_timeout = float(
            self.get_parameter("telemetry_reconnect_timeout_s").value
        )
        if reconnect_delay <= 0.0:
            raise ValueError("serial_reconnect_delay_s must be positive")
        if reconnect_timeout <= 0.25:
            raise ValueError("telemetry_reconnect_timeout_s must exceed 0.25 seconds")

        self._serial: serial.Serial | None = None
        self._serial_port: str | None = None
        self._connected_monotonic: float | None = None
        self._next_reconnect_monotonic = 0.0
        self._last_connection_warning = 0.0
        self._rx_buffer = bytearray()
        self._vision = SharedVisionTransport(str(self.get_parameter("vision_pty").value))
        self._rx_bytes_total = 0
        self._valid_frame_count = 0
        self._crc_error_count = 0
        self._discarded_byte_count = 0
        self._last_wrong_firmware_warning = 0.0
        self._last_motor_status: int | None = None
        self._command_sequence = 0
        self._latest_twist = Twist()
        self._last_twist_monotonic: float | None = None
        self._emergency_stop = False
        self._latest_gimbal = Vector3()
        self._last_gimbal_monotonic: float | None = None
        self._last_telemetry_monotonic: float | None = None
        self._last_mcu_time_ms: int | None = None
        self._last_stale_warning = 0.0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._odom_publisher = self.create_publisher(Odometry, odom_topic, 20)
        self._aim_yaw_online_publisher = self.create_publisher(
            Bool, "/sentry/aim_yaw_online", 10
        )
        self._aim_yaw_right_limit_publisher = self.create_publisher(
            Bool, "/sentry/aim_yaw_right_limit", 10
        )
        self._aim_yaw_left_limit_publisher = self.create_publisher(
            Bool, "/sentry/aim_yaw_left_limit", 10
        )
        self._aim_yaw_feedback_publisher = self.create_publisher(
            Float64MultiArray, "/sentry/aim_yaw_feedback", 10
        )
        self.create_subscription(Twist, command_topic, self._twist_callback, 20)
        self.create_subscription(Bool, "/emergency_stop", self._emergency_stop_callback, 10)
        self.create_subscription(Vector3, "/sentry/gimbal_manual", self._gimbal_callback, 20)
        self.create_timer(1.0 / command_rate_hz, self._send_command)
        self.create_timer(0.01, self._read_telemetry)
        self.create_timer(0.5, self._check_telemetry)
        self.get_logger().info(
            f"USB bridge ready: {command_topic} -> controller, telemetry -> {odom_topic}"
        )
        self._connect_serial()

    def _connect_serial(self) -> bool:
        now = time.monotonic()
        if self._serial is not None:
            return True
        if now < self._next_reconnect_monotonic:
            return False

        reconnect_delay = float(self.get_parameter("serial_reconnect_delay_s").value)
        try:
            port = choose_port(self._requested_port)
            connection = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=0,
                write_timeout=0,
                exclusive=True,
            )
        except (RuntimeError, OSError, serial.SerialException) as error:
            self._next_reconnect_monotonic = now + reconnect_delay
            if now - self._last_connection_warning >= 2.0:
                self.get_logger().warning(f"Waiting for USB controller: {error}")
                self._last_connection_warning = now
            return False

        try:
            self._vision.reset(now)
        except OSError as error:
            connection.close()
            raise RuntimeError(f"Cannot create vision PTY: {error}") from error
        self._last_twist_monotonic = None
        self._last_gimbal_monotonic = None
        self._serial = connection
        self._serial_port = port
        self._connected_monotonic = now
        self._next_reconnect_monotonic = 0.0
        self._rx_buffer.clear()
        self._last_telemetry_monotonic = None
        self._last_mcu_time_ms = None
        self._last_motor_status = None
        self.get_logger().info(f"USB controller connected on {port}")
        return True

    def _disconnect_serial(self, reason: str) -> None:
        self._vision.close()
        self._vision.tx.clear()
        self._vision.output.clear()
        self._vision.active = False
        self._last_twist_monotonic = None
        self._last_gimbal_monotonic = None
        connection = self._serial
        port = self._serial_port
        self._serial = None
        self._serial_port = None
        self._connected_monotonic = None
        self._last_telemetry_monotonic = None
        self._last_mcu_time_ms = None
        self._rx_buffer.clear()
        self._next_reconnect_monotonic = time.monotonic() + float(
            self.get_parameter("serial_reconnect_delay_s").value
        )
        if connection is not None:
            try:
                connection.close()
            except (OSError, serial.SerialException):
                pass
        self.get_logger().warning(
            f"USB controller disconnected from {port or 'unknown port'}: {reason}; "
            "automatic reconnect enabled"
        )

    def _twist_callback(self, message: Twist) -> None:
        if not self._emergency_stop:
            self._latest_twist = message
            self._last_twist_monotonic = time.monotonic()

    def _emergency_stop_callback(self, message: Bool) -> None:
        if message.data == self._emergency_stop:
            return
        self._emergency_stop = message.data
        self._last_twist_monotonic = None
        if self._emergency_stop and self._serial is not None:
            self._send_command(chassis_only=True)

    def _gimbal_callback(self, message: Vector3) -> None:
        self._latest_gimbal = message
        self._last_gimbal_monotonic = time.monotonic()

    def _send_command(self, *, chassis_only: bool = False) -> None:
        if not hasattr(self, "_vision"):
            # Keep offline diagnostic callers independent of a real PTY.
            self._vision = SharedVisionTransport(None)
        if self._serial is None and not self._connect_serial():
            return

        now = time.monotonic()
        timeout = float(self.get_parameter("command_timeout_s").value)
        enabled = (
            self._last_twist_monotonic is not None
            and now - self._last_twist_monotonic <= timeout
        )

        vx = float(self._latest_twist.linear.x) if enabled else 0.0
        vy = float(self._latest_twist.linear.y) if enabled else 0.0
        wz = float(self._latest_twist.angular.z) if enabled else 0.0
        if not all(math.isfinite(value) for value in (vx, vy, wz)):
            vx = vy = wz = 0.0
            enabled = False
        max_translation = float(self.get_parameter("max_translation_m_s").value)
        max_angular = float(self.get_parameter("max_angular_rad_s").value)
        translation = math.hypot(vx, vy)
        if translation > max_translation > 0.0:
            scale = max_translation / translation
            vx *= scale
            vy *= scale
        wz = max(-max_angular, min(max_angular, wz))

        body = COMMAND_BODY.pack(
            MAGIC,
            PROTOCOL_VERSION,
            COMMAND_TYPE,
            self._command_sequence,
            round(vx * 1000.0),
            round(vy * 1000.0),
            round(wz * 1000.0),
            COMMAND_ENABLE if enabled else 0,
        )
        self._command_sequence = (self._command_sequence + 1) & 0xFFFF
        frame = body + CRC.pack(crc16_ccitt(body))
        connection = self._serial
        if connection is None:
            return
        try:
            self._vision.tx.put(frame, now, "nav")
            if not chassis_only:
                gimbal_fresh = (
                    self._last_gimbal_monotonic is not None
                    and now - self._last_gimbal_monotonic <= timeout
                )
                gimbal = self._latest_gimbal
                yaw_rate = float(gimbal.x) if gimbal_fresh else 0.0
                pitch_rate = float(gimbal.y) if gimbal_fresh else 0.0
                if not math.isfinite(yaw_rate) or not math.isfinite(pitch_rate):
                    yaw_rate = pitch_rate = 0.0
                    gimbal_fresh = False
                spin = gimbal_fresh and gimbal.z >= 0.5
                gimbal_body = COMMAND_BODY.pack(
                    MAGIC, PROTOCOL_VERSION, GIMBAL_COMMAND_TYPE,
                    self._command_sequence, round(max(-0.60, min(0.60, yaw_rate)) * 1000),
                    round(max(-0.20, min(0.20, pitch_rate)) * 1000), 0,
                    (COMMAND_ENABLE if gimbal_fresh else 0) | (0x02 if spin else 0),
                )
                gimbal_frame = gimbal_body + CRC.pack(crc16_ccitt(gimbal_body))
                self._vision.tx.put(gimbal_frame, now, "manual")
            for _ in range(4):
                self._vision.tx.flush(connection.write, now)
        except (OSError, serial.SerialException, BufferError, TimeoutError) as error:
            self._disconnect_serial(f"command write failed: {error}")

    def _read_telemetry(self) -> None:
        if not hasattr(self, "_vision"):
            # Keep offline diagnostic callers independent of a real PTY.
            self._vision = SharedVisionTransport(None)
        connection = self._serial
        if connection is None:
            return
        now = time.monotonic()
        try:
            self._vision.poll(now)
            waiting = min(connection.in_waiting, 4096)
            received = connection.read(waiting) if waiting else b""
            self._rx_bytes_total += len(received)
            frames = self._vision.feed_usb(received, now)
            for _ in range(4):
                self._vision.tx.flush(connection.write, now)
        except (OSError, serial.SerialException, BufferError, TimeoutError) as error:
            self._disconnect_serial(f"shared serial transport failed: {error}")
            return
        self._crc_error_count = self._vision.parser.crc_errors
        self._discarded_byte_count = self._vision.parser.discarded
        for kind, frame in frames:
            body = frame[:-CRC.size]
            if body[3] == AIM_TELEMETRY_TYPE:
                fields = AIM_TELEMETRY_BODY.unpack(body)
                # Do not refresh online or limit feedback from malformed data.
                if (fields[5] >= ECD_FULL_RANGE or
                        (fields[10] & 0xE0 and not fields[10] & 0x10) or
                        not all(math.isfinite(value) for value in fields[6:9])):
                    self._discarded_byte_count += TELEMETRY_FRAME_SIZE
                    continue
                self._valid_frame_count += 1
                self._handle_aim_telemetry(fields)
                continue

            fields = TELEMETRY_BODY.unpack(body)
            self._valid_frame_count += 1
            mcu_time_ms = int(fields[4])
            steer_ecd = tuple(int(value) for value in fields[5:9])
            drive_rpm = tuple(int(value) for value in fields[9:13])
            motor_status = int(fields[13])
            aim_feedback_supported = bool(
                motor_status & STATUS_AIM_YAW_FEEDBACK_SUPPORTED
            )
            if aim_feedback_supported:
                # Directional limits require the dedicated two-edge frame.
                # Older firmware may report online, but cannot permit motion.
                aim_yaw_online = Bool()
                aim_yaw_online.data = bool(motor_status & STATUS_AIM_YAW_ONLINE)
                self._aim_yaw_online_publisher.publish(aim_yaw_online)
            if motor_status != self._last_motor_status:
                self._last_motor_status = motor_status
                online = lambda bit: "yes" if motor_status & bit else "no"
                aim_yaw_status = (
                    online(STATUS_AIM_YAW_ONLINE) if aim_feedback_supported else "unknown"
                )
                self.get_logger().info(
                    f"controller: chassis={online(0x01)} aim_yaw_online={aim_yaw_status} "
                    f"aim_yaw_right_limit={online(STATUS_AIM_YAW_RIGHT_LIMIT)} "
                    f"lower_yaw_online={online(0x08)} "
                    f"lower_yaw_enabled={online(0x10)} spin_request={online(0x20)}"
                )
            self._handle_telemetry(mcu_time_ms, steer_ecd, drive_rpm)

    def _handle_aim_telemetry(self, fields: tuple) -> None:
        flags = int(fields[10])
        left_limit = Bool()
        left_limit.data = bool(flags & 0x02)
        self._aim_yaw_left_limit_publisher.publish(left_limit)
        right_limit = Bool()
        right_limit.data = bool(flags & 0x04)
        self._aim_yaw_right_limit_publisher.publish(right_limit)
        online = Bool()
        online.data = bool(flags & 0x01)
        self._aim_yaw_online_publisher.publish(online)
        feedback = Float64MultiArray()
        # Encoder, actual angle, target angle, speed, output, flags, MCU time.
        feedback.data = [float(value) for value in (*fields[5:11], fields[4])]
        self._aim_yaw_feedback_publisher.publish(feedback)

    def _handle_telemetry(
        self,
        mcu_time_ms: int,
        steer_ecd: tuple[int, ...],
        drive_rpm: tuple[int, ...],
    ) -> None:
        now_monotonic = time.monotonic()
        telemetry_was_stale = (
            self._last_telemetry_monotonic is None
            or now_monotonic - self._last_telemetry_monotonic > 0.25
        )
        dt = 0.0
        if self._last_mcu_time_ms is not None:
            elapsed_ms = (mcu_time_ms - self._last_mcu_time_ms) & 0xFFFFFFFF
            if 0 < elapsed_ms <= 250:
                dt = elapsed_ms * 0.001
        self._last_mcu_time_ms = mcu_time_ms
        self._last_telemetry_monotonic = now_monotonic
        if telemetry_was_stale:
            self.get_logger().info(
                f"Controller telemetry active on {self._serial_port}: "
                f"valid={self._valid_frame_count}, crc_errors={self._crc_error_count}"
            )

        wheel_vx: list[float] = []
        wheel_vy: list[float] = []
        for index in range(MOTOR_COUNT):
            ecd_delta = (steer_ecd[index] - STEER_ZERO_ECD[index] + ECD_FULL_RANGE // 2)
            ecd_delta = ecd_delta % ECD_FULL_RANGE - ECD_FULL_RANGE // 2
            hardware_angle = (
                ecd_delta
                * STEER_DIRECTION_SIGN[index]
                * 2.0
                * math.pi
                / ECD_FULL_RANGE
            )
            ros_angle = -hardware_angle
            logical_rpm = drive_rpm[index] * DRIVE_FEEDBACK_SIGN[index]
            wheel_speed = (
                logical_rpm
                / WHEEL_REDUCTION_RATIO
                * 2.0
                * math.pi
                * WHEEL_RADIUS_M
                / 60.0
            )
            wheel_vx.append(wheel_speed * math.cos(ros_angle))
            wheel_vy.append(wheel_speed * math.sin(ros_angle))

        vx = sum(wheel_vx) / MOTOR_COUNT
        vy = sum(wheel_vy) / MOTOR_COUNT
        rotation_denominator = sum(
            x_pos * x_pos + y_pos * y_pos
            for x_pos, y_pos in zip(MODULE_X_M, MODULE_Y_M)
        )
        wz = sum(
            -MODULE_Y_M[index] * wheel_vx[index]
            + MODULE_X_M[index] * wheel_vy[index]
            for index in range(MOTOR_COUNT)
        ) / rotation_denominator

        if dt > 0.0:
            cos_yaw = math.cos(self._yaw)
            sin_yaw = math.sin(self._yaw)
            self._x += (vx * cos_yaw - vy * sin_yaw) * dt
            self._y += (vx * sin_yaw + vy * cos_yaw) * dt
            self._yaw = wrap_angle(self._yaw + wz * dt)

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self._x
        message.pose.pose.position.y = self._y
        message.pose.pose.orientation.z = math.sin(0.5 * self._yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * self._yaw)
        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = wz
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.04
        message.twist.covariance[0] = 0.03
        message.twist.covariance[7] = 0.03
        message.twist.covariance[35] = 0.05
        self._odom_publisher.publish(message)

    def _check_telemetry(self) -> None:
        now = time.monotonic()
        if self._serial is None:
            self._connect_serial()
            return

        reference_time = self._last_telemetry_monotonic
        if reference_time is None:
            reference_time = self._connected_monotonic
        if reference_time is None:
            return

        telemetry_age = now - reference_time
        if telemetry_age > 0.25:
            if now - self._last_stale_warning > 2.0:
                self.get_logger().warning(
                    "No fresh controller telemetry; /wheel/odom is stale "
                    f"(age={telemetry_age:.2f}s, rx_bytes={self._rx_bytes_total}, "
                    f"valid={self._valid_frame_count}, crc_errors={self._crc_error_count}, "
                    f"discarded={self._discarded_byte_count})"
                )
                self._last_stale_warning = now

        reconnect_timeout = float(
            self.get_parameter("telemetry_reconnect_timeout_s").value
        )
        if telemetry_age >= reconnect_timeout:
            self._disconnect_serial(
                f"telemetry remained stale for {telemetry_age:.2f} seconds"
            )

    def destroy_node(self) -> bool:
        self._vision.close()
        connection = self._serial
        try:
            if connection is not None:
                zero_body = COMMAND_BODY.pack(
                    MAGIC,
                    PROTOCOL_VERSION,
                    COMMAND_TYPE,
                    self._command_sequence,
                    0,
                    0,
                    0,
                    0,
                )
                zero_frame = zero_body + CRC.pack(crc16_ccitt(zero_body))
                gimbal_zero_body = COMMAND_BODY.pack(
                    MAGIC,
                    PROTOCOL_VERSION,
                    GIMBAL_COMMAND_TYPE,
                    self._command_sequence,
                    0,
                    0,
                    0,
                    0,
                )
                gimbal_zero_frame = gimbal_zero_body + CRC.pack(crc16_ccitt(gimbal_zero_body))
                now = time.monotonic()
                self._vision.tx.cancel_unsent()
                self._vision.tx.put(RELEASE, now, "vision")
                self._vision.tx.put(zero_frame, now, "nav")
                self._vision.tx.put(gimbal_zero_frame, now, "manual")
                for _ in range(128):
                    if not self._vision.tx.pending or time.monotonic() - now > .05:
                        break
                    self._vision.tx.flush(connection.write, time.monotonic())
        except (OSError, serial.SerialException, BufferError, TimeoutError):
            pass
        finally:
            if connection is not None:
                try:
                    connection.close()
                except (OSError, serial.SerialException):
                    pass
        self._serial = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SentryNav2Bridge | None = None
    try:
        node = SentryNav2Bridge()
        rclpy.spin(node)
    except (RuntimeError, OSError, serial.SerialException, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"sentry_nav2_bridge: {error}")
        raise SystemExit(1) from error
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
