#!/usr/bin/env python3
"""Serve a browser UI that publishes safe key-down/key-up teleop commands."""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import subprocess
import threading
import time
from typing import Any
from urllib.parse import parse_qs, quote, urlparse

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, Vector3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64MultiArray


CHASSIS_KEYS = frozenset({"w", "a", "s", "d", "q", "e"})
SMALL_YAW_KEYS = frozenset({"j", "l"})
ALLOWED_KEYS = CHASSIS_KEYS | SMALL_YAW_KEYS
MAX_REQUEST_BODY = 4096
ZERO_BURST_FRAMES = 4
SMALL_YAW_RATE_RAD_S = 0.60
CONTROLLER_STATUS_TIMEOUT_S = 0.25
# B toggles beyblade mode: the chassis spins at this fixed rate while W/A/S/D
# still drive translation on top of it, and the lower yaw is asked to
# counter-spin (see _publish_tick). This is below the standalone manual
# controller's 2.4 rad/s command ceiling.
BEYBLADE_SPIN_RATE_RAD_S = 0.50


class SharedControlState:
    """Thread-safe browser ownership and key state with a deadman timer."""

    def __init__(self, watchdog_s: float) -> None:
        self._watchdog_s = watchdog_s
        self._lock = threading.Lock()
        self._active_client: str | None = None
        self._last_sequence = -1
        self._last_seen = 0.0
        self._armed = False
        self._keys: set[str] = set()
        self._speed_scale = 1.0
        self._spin_enabled = False
        self._small_yaw_armed = False
        self._small_yaw_online: bool | None = None
        self._small_yaw_status_seen: float | None = None
        self._small_yaw_right_limit: bool | None = None
        self._small_yaw_limit_seen: float | None = None
        self._small_yaw_left_limit: bool | None = None
        self._small_yaw_left_limit_seen: float | None = None
        self._small_yaw_diagnostics: dict[str, float] = {}
        self._small_yaw_diagnostic_seen: float | None = None
        self._chassis_diagnostics: dict[str, bool | None] = {}

    @staticmethod
    def _validate_client(client: Any) -> str:
        if not isinstance(client, str) or not 8 <= len(client) <= 128:
            raise ValueError("client must be an 8-128 character identifier")
        return client

    def claim(self, client: Any) -> dict[str, Any]:
        client_id = self._validate_client(client)
        with self._lock:
            self._active_client = client_id
            self._last_sequence = -1
            self._last_seen = time.monotonic()
            self._armed = False
            self._keys.clear()
            self._spin_enabled = False
            self._small_yaw_armed = False
            return self._snapshot_locked(time.monotonic())

    def update(
        self,
        client: Any,
        sequence: Any,
        armed: Any,
        keys: Any,
        speed_scale: Any,
        spin_enabled: Any = False,
        small_yaw_armed: Any = False,
    ) -> tuple[bool, str, dict[str, Any]]:
        client_id = self._validate_client(client)
        if not isinstance(sequence, int) or sequence < 0:
            raise ValueError("sequence must be a non-negative integer")
        if not isinstance(armed, bool):
            raise ValueError("armed must be a boolean")
        if not isinstance(spin_enabled, bool):
            raise ValueError("spin_enabled must be a boolean")
        if not isinstance(small_yaw_armed, bool):
            raise ValueError("small_yaw_armed must be a boolean")
        if not isinstance(keys, list) or any(not isinstance(key, str) for key in keys):
            raise ValueError("keys must be a string array")
        requested_keys = {key.lower() for key in keys}
        if not requested_keys.issubset(ALLOWED_KEYS):
            raise ValueError("keys contains an unsupported control")
        try:
            requested_scale = float(speed_scale)
        except (TypeError, ValueError) as error:
            raise ValueError("speed_scale must be numeric") from error
        if not math.isfinite(requested_scale):
            raise ValueError("speed_scale must be finite")
        requested_scale = max(0.2, min(1.0, requested_scale))

        now = time.monotonic()
        with self._lock:
            self._expire_locked(now)
            if client_id != self._active_client:
                return False, "control ownership is not held by this page", self._snapshot_locked(now)
            if sequence <= self._last_sequence:
                return True, "stale update ignored", self._snapshot_locked(now)

            self._last_sequence = sequence
            self._last_seen = now
            self._armed = armed
            self._small_yaw_armed = (
                small_yaw_armed and self._small_yaw_online_locked(now) is True
            )
            enabled_keys = (CHASSIS_KEYS if armed else frozenset()) | (
                SMALL_YAW_KEYS if self._small_yaw_armed else frozenset()
            )
            self._keys = requested_keys & enabled_keys
            self._filter_yaw_limits_locked(now)
            self._speed_scale = requested_scale
            self._spin_enabled = spin_enabled if armed else False
            return True, "updated", self._snapshot_locked(now)

    def release(self, client: Any) -> dict[str, Any]:
        client_id = self._validate_client(client)
        with self._lock:
            if client_id == self._active_client:
                self._active_client = None
                self._last_sequence = -1
                self._last_seen = 0.0
                self._armed = False
                self._keys.clear()
                self._spin_enabled = False
                self._small_yaw_armed = False
            return self._snapshot_locked(time.monotonic())

    def set_small_yaw_online(self, online: bool) -> None:
        with self._lock:
            now = time.monotonic()
            self._small_yaw_online = online
            self._small_yaw_status_seen = now
            self._expire_locked(now)

    def _small_yaw_online_locked(self, now: float) -> bool | None:
        if self._small_yaw_status_seen is None or (
            now - self._small_yaw_status_seen > CONTROLLER_STATUS_TIMEOUT_S
        ):
            return None
        return self._small_yaw_online

    def set_small_yaw_right_limit(self, at_limit: bool) -> None:
        with self._lock:
            now = time.monotonic()
            self._small_yaw_right_limit = at_limit
            self._small_yaw_limit_seen = now
            self._filter_yaw_limits_locked(now)

    def set_small_yaw_left_limit(self, at_limit: bool) -> None:
        with self._lock:
            now = time.monotonic()
            self._small_yaw_left_limit = at_limit
            self._small_yaw_left_limit_seen = now
            self._filter_yaw_limits_locked(now)

    def set_small_yaw_feedback(self, values: list[float]) -> bool:
        """Store a validated two-edge sample and its diagnostics atomically."""
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            return False
        ecd, actual, target, speed, output, flags, mcu_ms = values
        if (ecd != int(ecd) or not 0 <= ecd < 8192 or
                flags != int(flags) or not 0 <= flags <= 0xFF or
                (int(flags) & 0xE0 and not int(flags) & 0x10) or
                output != int(output) or not -32768 <= output <= 32767 or
                mcu_ms != int(mcu_ms) or not 0 <= mcu_ms <= 0xFFFFFFFF):
            return False
        with self._lock:
            now = time.monotonic()
            self._small_yaw_online = bool(int(flags) & 0x01)
            self._small_yaw_status_seen = now
            self._small_yaw_left_limit = bool(int(flags) & 0x02)
            self._small_yaw_left_limit_seen = now
            self._small_yaw_right_limit = bool(int(flags) & 0x04)
            self._small_yaw_limit_seen = now
            self._small_yaw_diagnostic_seen = now
            self._small_yaw_diagnostics = {
                "small_yaw_ecd": int(ecd),
                "small_yaw_actual_deg": actual,
                "small_yaw_target_deg": target,
                "small_yaw_error_deg": target - actual,
                "small_yaw_speed_deg_s": speed,
                "small_yaw_output": int(output),
                "small_yaw_controller_active": bool(int(flags) & 0x08),
                "small_yaw_mcu_time_ms": int(mcu_ms),
            }
            chassis_supported = bool(int(flags) & 0x10)
            self._chassis_diagnostics = {
                "chassis_steer_online": bool(int(flags) & 0x20) if chassis_supported else None,
                "chassis_drive_online": bool(int(flags) & 0x40) if chassis_supported else None,
                "chassis_control_enabled": bool(int(flags) & 0x80) if chassis_supported else None,
            }
            self._expire_locked(now)
        return True

    def _small_yaw_left_limit_locked(self, now: float) -> bool | None:
        if self._small_yaw_left_limit_seen is None or (
            now - self._small_yaw_left_limit_seen > CONTROLLER_STATUS_TIMEOUT_S
        ):
            return None
        return self._small_yaw_left_limit

    def _small_yaw_right_limit_locked(self, now: float) -> bool | None:
        if self._small_yaw_limit_seen is None or (
            now - self._small_yaw_limit_seen > CONTROLLER_STATUS_TIMEOUT_S
        ):
            return None
        return self._small_yaw_right_limit

    def _filter_yaw_limits_locked(self, now: float) -> None:
        if ((self._small_yaw_right_limit_locked(now) is not False and "l" in self._keys) or
                (self._small_yaw_left_limit_locked(now) is not False and "j" in self._keys)):
            # Opposing keys remain a zero request, including at the limit.
            self._keys.discard("l")
            self._keys.discard("j")

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            self._expire_locked(now)
            return self._snapshot_locked(now)

    def _expire_locked(self, now: float) -> None:
        if (self._armed or self._small_yaw_armed) and now - self._last_seen > self._watchdog_s:
            self._active_client = None
            self._last_sequence = -1
            self._last_seen = 0.0
            self._armed = False
            self._keys.clear()
            self._spin_enabled = False
            self._small_yaw_armed = False
        if self._small_yaw_armed and self._small_yaw_online_locked(now) is not True:
            self._small_yaw_armed = False
            self._keys.difference_update(SMALL_YAW_KEYS)
        self._filter_yaw_limits_locked(now)

    def _snapshot_locked(self, now: float) -> dict[str, Any]:
        age_s = None if self._last_seen == 0.0 else max(0.0, now - self._last_seen)
        diagnostic_age = (
            None if self._small_yaw_diagnostic_seen is None
            else max(0.0, now - self._small_yaw_diagnostic_seen)
        )
        diagnostics_fresh = (
            diagnostic_age is not None and diagnostic_age <= CONTROLLER_STATUS_TIMEOUT_S
        )
        chassis = {
            key: self._chassis_diagnostics.get(key) if diagnostics_fresh else None
            for key in ("chassis_steer_online", "chassis_drive_online", "chassis_control_enabled")
        }
        steer_online = chassis["chassis_steer_online"]
        drive_online = chassis["chassis_drive_online"]
        chassis_ready = (
            steer_online and drive_online
            if steer_online is not None and drive_online is not None else None
        )
        if chassis_ready is None:
            chassis_reason = "等待底盘在线反馈"
        elif not steer_online and not drive_online:
            chassis_reason = "舵向与驱动电机未全部在线"
        elif not steer_online:
            chassis_reason = "舵向电机未全部在线"
        elif not drive_online:
            chassis_reason = "驱动电机未全部在线"
        else:
            chassis_reason = "底盘电机全部在线"
        return {
            "active_client": self._active_client,
            "armed": self._armed,
            **chassis,
            "chassis_ready": chassis_ready,
            "chassis_block_reason": chassis_reason,
            "small_yaw_armed": self._small_yaw_armed,
            "small_yaw_online": self._small_yaw_online_locked(now),
            "small_yaw_right_limit": self._small_yaw_right_limit_locked(now),
            "small_yaw_left_limit": self._small_yaw_left_limit_locked(now),
            "small_yaw_left_limit_feedback_age_s": (
                None if self._small_yaw_left_limit_seen is None
                else max(0.0, now - self._small_yaw_left_limit_seen)
            ),
            "small_yaw_diagnostic_age_s": diagnostic_age,
            **{
                key: self._small_yaw_diagnostics.get(key) if diagnostics_fresh else None
                for key in (
                    "small_yaw_ecd", "small_yaw_actual_deg", "small_yaw_target_deg",
                    "small_yaw_error_deg", "small_yaw_speed_deg_s", "small_yaw_output",
                    "small_yaw_controller_active", "small_yaw_mcu_time_ms",
                )
            },
            "small_yaw_limit_feedback_age_s": (
                None if self._small_yaw_limit_seen is None
                else max(0.0, now - self._small_yaw_limit_seen)
            ),
            "small_yaw_feedback_age_s": (
                None if self._small_yaw_status_seen is None
                else max(0.0, now - self._small_yaw_status_seen)
            ),
            "keys": sorted(self._keys),
            "spin_enabled": self._spin_enabled,
            "lower_spin_available": True,
            "speed_scale": self._speed_scale,
            "heartbeat_age_s": age_s,
            "watchdog_s": self._watchdog_s,
        }


class BrowserTeleop(Node):
    """Publish browser key state while explicitly armed."""

    def __init__(
        self,
        topic: str,
        linear_speed: float,
        angular_speed: float,
        watchdog_s: float,
        publish_rate_hz: float,
    ) -> None:
        super().__init__("wasd_web_teleop")
        self.control_state = SharedControlState(watchdog_s)
        self._publisher = self.create_publisher(Twist, topic, 10)
        self._manual_override_publisher = self.create_publisher(
            Bool,
            "/navigation_manual_override",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._manual_override = False
        self._manual_override_publisher.publish(Bool(data=False))
        self._gimbal_publisher = self.create_publisher(Vector3, "/sentry/gimbal_manual", 10)
        self.create_subscription(
            Bool, "/sentry/aim_yaw_online", self._small_yaw_status_callback, 10
        )
        self.create_subscription(
            Bool, "/sentry/aim_yaw_right_limit", self._small_yaw_limit_callback, 10
        )
        self.create_subscription(
            Bool, "/sentry/aim_yaw_left_limit", self._small_yaw_left_limit_callback, 10
        )
        self.create_subscription(
            Float64MultiArray, "/sentry/aim_yaw_feedback", self._small_yaw_feedback_callback, 10
        )
        self._linear_speed = linear_speed
        self._angular_speed = angular_speed
        self._was_armed = False
        self._zero_frames_remaining = 0
        self._was_gimbal_armed = False
        self._gimbal_zero_frames_remaining = 0
        self.create_timer(1.0 / publish_rate_hz, self._publish_tick)
        self.get_logger().info(
            f"Browser teleop ready: topic={topic}, linear={linear_speed:.2f} m/s, "
            f"angular={angular_speed:.2f} rad/s, watchdog={watchdog_s:.2f} s"
        )

    @property
    def limits(self) -> dict[str, float | bool]:
        return {
            "linear_speed": self._linear_speed,
            "angular_speed": self._angular_speed,
            "lower_spin_available": True,
            "beyblade_spin_rate": BEYBLADE_SPIN_RATE_RAD_S,
            "small_yaw_rate": SMALL_YAW_RATE_RAD_S,
        }

    def _small_yaw_status_callback(self, message: Bool) -> None:
        self.control_state.set_small_yaw_online(bool(message.data))

    def _small_yaw_limit_callback(self, message: Bool) -> None:
        self.control_state.set_small_yaw_right_limit(bool(message.data))

    def _small_yaw_left_limit_callback(self, message: Bool) -> None:
        self.control_state.set_small_yaw_left_limit(bool(message.data))

    def _small_yaw_feedback_callback(self, message: Float64MultiArray) -> None:
        self.control_state.set_small_yaw_feedback(message.data)

    def _set_manual_override(self, active: bool) -> None:
        if self._manual_override != active:
            self._manual_override = active
            self._manual_override_publisher.publish(Bool(data=active))

    def _publish_tick(self) -> None:
        snapshot = self.control_state.snapshot()
        keys = set(snapshot["keys"])
        scale = float(snapshot["speed_scale"])
        armed = bool(snapshot["armed"])
        small_yaw_armed = bool(snapshot["small_yaw_armed"])

        vx_scale = float(("w" in keys) - ("s" in keys))
        vy_scale = float(("a" in keys) - ("d" in keys))
        wz_scale = float(("q" in keys) - ("e" in keys))
        translation_norm = math.hypot(vx_scale, vy_scale)
        if translation_norm > 1.0:
            vx_scale /= translation_norm
            vy_scale /= translation_norm

        beyblading = bool(snapshot["spin_enabled"])

        if armed:
            message = Twist()
            message.linear.x = vx_scale * self._linear_speed * scale
            message.linear.y = vy_scale * self._linear_speed * scale
            # Beyblade mode: spin the chassis at a fixed rate regardless of
            # Q/E, while W/A/S/D translation above keeps working, so the
            # robot can move around while spinning. Toggle B off to get
            # direct Q/E control back.
            message.angular.z = (
                BEYBLADE_SPIN_RATE_RAD_S if beyblading else wz_scale * self._angular_speed * scale
            )
            if message.linear.x or message.linear.y or message.angular.z:
                self._set_manual_override(True)
            self._publisher.publish(message)
            self._was_armed = True
            self._zero_frames_remaining = 0
        else:
            self._set_manual_override(False)
            if self._was_armed:
                self._zero_frames_remaining = ZERO_BURST_FRAMES
                self._was_armed = False
            if self._zero_frames_remaining > 0:
                self._publisher.publish(Twist())
                self._zero_frames_remaining -= 1

        if armed or small_yaw_armed:
            # Lower yaw is asked to counter-spin whenever beyblade mode is
            # on. This is sent unconditionally -- the firmware already
            # no-ops safely if that motor isn't online, so we don't gate
            # the request on hardware presence here.
            gimbal = Vector3()
            if small_yaw_armed:
                gimbal.x = float(("j" in keys) - ("l" in keys)) * SMALL_YAW_RATE_RAD_S * scale
            gimbal.z = 1.0 if beyblading else 0.0
            self._gimbal_publisher.publish(gimbal)
            self._was_gimbal_armed = True
            self._gimbal_zero_frames_remaining = 0
            return

        if self._was_gimbal_armed:
            self._gimbal_zero_frames_remaining = ZERO_BURST_FRAMES
            self._was_gimbal_armed = False
        if self._gimbal_zero_frames_remaining > 0:
            self._gimbal_publisher.publish(Vector3())
            self._gimbal_zero_frames_remaining -= 1

    def publish_shutdown_stop(self) -> None:
        if not rclpy.ok():
            return
        stop_chassis = self._was_armed or self._zero_frames_remaining > 0
        stop_gimbal = self._was_gimbal_armed or self._gimbal_zero_frames_remaining > 0
        for _ in range(ZERO_BURST_FRAMES):
            if stop_chassis:
                self._publisher.publish(Twist())
            if stop_gimbal:
                self._gimbal_publisher.publish(Vector3())
            time.sleep(0.02)
        self._set_manual_override(False)


class TeleopHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        handler: type[BaseHTTPRequestHandler],
        node: BrowserTeleop,
        token: str,
        page: bytes,
    ) -> None:
        self.teleop_node = node
        self.access_token = token
        self.page = page
        super().__init__(address, handler)


class TeleopRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server: TeleopHttpServer

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _route(self) -> tuple[str, dict[str, list[str]]]:
        parsed = urlparse(self.path)
        return parsed.path, parse_qs(parsed.query)

    def _authorized(self, query: dict[str, list[str]]) -> bool:
        if not self.server.access_token:
            return True
        supplied = query.get("token", [""])[0]
        return bool(supplied) and secrets.compare_digest(supplied, self.server.access_token)

    def _same_origin(self) -> bool:
        origin = self.headers.get("Origin")
        if not origin:
            return True
        parsed = urlparse(origin)
        return (
            parsed.scheme in {"http", "https"}
            and parsed.netloc == self.headers.get("Host", "")
        )

    def _send_bytes(self, status: int, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; style-src 'self' 'unsafe-inline'; "
            "script-src 'self' 'unsafe-inline'; connect-src 'self'",
        )
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._send_bytes(status, "application/json; charset=utf-8", body)

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ValueError("invalid Content-Length") from error
        if length <= 0 or length > MAX_REQUEST_BODY:
            raise ValueError("request body length is invalid")
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError("request body must be valid JSON") from error
        if not isinstance(payload, dict):
            raise ValueError("request body must be a JSON object")
        return payload

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path, query = self._route()
        if path == "/healthz":
            self._send_json(200, {"ok": True})
            return
        if not self._authorized(query):
            self._send_json(403, {"ok": False, "error": "invalid access token"})
            return
        if path == "/":
            self._send_bytes(200, "text/html; charset=utf-8", self.server.page)
            return
        if path == "/api/state":
            state = self.server.teleop_node.control_state.snapshot()
            state.update(self.server.teleop_node.limits)
            self._send_json(200, {"ok": True, "state": state})
            return
        self._send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path, query = self._route()
        if not self._same_origin():
            self._send_json(403, {"ok": False, "error": "cross-origin request rejected"})
            return
        if not self._authorized(query):
            self._send_json(403, {"ok": False, "error": "invalid access token"})
            return
        try:
            payload = self._read_json()
            if path == "/api/claim":
                state = self.server.teleop_node.control_state.claim(payload.get("client"))
                state.update(self.server.teleop_node.limits)
                self._send_json(200, {"ok": True, "state": state})
                return
            if path == "/api/control":
                accepted, message, state = self.server.teleop_node.control_state.update(
                    payload.get("client"),
                    payload.get("sequence"),
                    payload.get("armed"),
                    payload.get("keys"),
                    payload.get("speed_scale", 1.0),
                    payload.get("spin_enabled", False),
                    payload.get("small_yaw_armed", False),
                )
                state.update(self.server.teleop_node.limits)
                status = 200 if accepted else 409
                self._send_json(status, {"ok": accepted, "message": message, "state": state})
                return
            if path == "/api/release":
                state = self.server.teleop_node.control_state.release(payload.get("client"))
                state.update(self.server.teleop_node.limits)
                self._send_json(200, {"ok": True, "state": state})
                return
            self._send_json(404, {"ok": False, "error": "not found"})
        except ValueError as error:
            self._send_json(400, {"ok": False, "error": str(error)})


def parse_arguments() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind address")
    parser.add_argument("--port", type=int, default=8088, help="HTTP port")
    parser.add_argument("--topic", default="/cmd_vel_teleop")
    parser.add_argument("--speed", type=float, default=0.80, help="maximum translation m/s")
    parser.add_argument("--turn", type=float, default=2.40, help="maximum yaw rad/s")
    parser.add_argument("--watchdog", type=float, default=0.35, help="browser heartbeat timeout")
    parser.add_argument("--rate", type=float, default=30.0, help="ROS publish rate in Hz")
    parser.add_argument("--token", default="", help="optional fixed access token")
    return parser.parse_known_args()


def validate_arguments(arguments: argparse.Namespace) -> None:
    if not arguments.host:
        raise ValueError("host must not be empty")
    if not 1 <= arguments.port <= 65535:
        raise ValueError("port must be between 1 and 65535")
    if not arguments.topic:
        raise ValueError("topic must not be empty")
    if arguments.speed <= 0.0 or arguments.turn <= 0.0:
        raise ValueError("speed and turn must be positive")
    if arguments.watchdog < 0.20:
        raise ValueError("watchdog must be at least 0.20 seconds")
    if arguments.rate < 10.0:
        raise ValueError("rate must be at least 10 Hz")


def host_urls(port: int, token: str) -> list[str]:
    addresses: list[str] = []
    try:
        result = subprocess.run(
            ["hostname", "-I"], capture_output=True, check=False, text=True, timeout=2.0
        )
        for address in result.stdout.split():
            if ":" not in address and address not in addresses and not address.startswith("172.17."):
                addresses.append(address)
    except (OSError, subprocess.SubprocessError):
        pass
    if not addresses:
        addresses.append("127.0.0.1")
    suffix = f"?token={quote(token, safe='')}" if token else ""
    return [f"http://{address}:{port}/{suffix}" for address in addresses]


def main() -> int:
    arguments, ros_arguments = parse_arguments()
    try:
        validate_arguments(arguments)
    except ValueError as error:
        print(f"wasd_web_teleop: {error}")
        return 2

    rclpy.init(args=ros_arguments, signal_handler_options=SignalHandlerOptions.NO)
    node: BrowserTeleop | None = None
    server: TeleopHttpServer | None = None
    server_thread: threading.Thread | None = None
    try:
        node = BrowserTeleop(
            topic=arguments.topic,
            linear_speed=arguments.speed,
            angular_speed=arguments.turn,
            watchdog_s=arguments.watchdog,
            publish_rate_hz=arguments.rate,
        )
        page_path = (
            Path(get_package_share_directory("mid360_nav_bringup"))
            / "web"
            / "wasd.html"
        )
        token = arguments.token
        server = TeleopHttpServer(
            (arguments.host, arguments.port),
            TeleopRequestHandler,
            node,
            token,
            page_path.read_bytes(),
        )
        server_thread = threading.Thread(
            target=server.serve_forever,
            kwargs={"poll_interval": 0.1},
            name="wasd-http",
            daemon=True,
        )
        server_thread.start()

        node.get_logger().info("Open the browser controller on the Windows computer:")
        for url in host_urls(arguments.port, token):
            node.get_logger().info(f"  {url}")
        if not token:
            node.get_logger().warning(
                "Access token disabled: any device on the reachable LAN can open the controller."
            )
        node.get_logger().info("Enable chassis for W/A/S/D/Q/E; enable small yaw separately for J/L.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"wasd_web_teleop: {error}")
        return 1
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if server_thread is not None:
            server_thread.join(timeout=1.0)
        if node is not None:
            node.publish_shutdown_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
