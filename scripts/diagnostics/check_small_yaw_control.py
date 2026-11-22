#!/usr/bin/env python3
"""Exercise small-yaw control without opening a serial port or ROS node.

Source the ROS environment, then run through ``just _py``. All publishers,
serial connections, parameters, and clocks below are test doubles.
"""

import copy
from contextlib import redirect_stdout
import ctypes
import importlib.util
import io
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / "ros2_ws/src/mid360_nav_bringup"
# The tooling venv does not inherit apt-installed ROS message dependencies.
# Keep its own packages first while making the host's NumPy available.
SYSTEM_PACKAGES = Path("/usr/lib/python3/dist-packages")
if SYSTEM_PACKAGES.is_dir() and str(SYSTEM_PACKAGES) not in sys.path:
    sys.path.append(str(SYSTEM_PACKAGES))


def load_script(name, filename):
    spec = importlib.util.spec_from_file_location(name, BRINGUP / "scripts" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


web = load_script("small_yaw_web_under_test", "wasd_web_teleop.py")
bridge = load_script("small_yaw_bridge_under_test", "sentry_nav2_bridge.py")
diagnostic_spec = importlib.util.spec_from_file_location(
    "small_yaw_diagnostic_under_test", ROOT / "scripts/diagnostics/small_yaw_status.py"
)
diagnostic = importlib.util.module_from_spec(diagnostic_spec)
diagnostic_spec.loader.exec_module(diagnostic)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class FakeSerial:
    def __init__(self, received=b""):
        self.received = bytearray(received)
        self.written = []

    @property
    def in_waiting(self):
        return len(self.received)

    def read(self, length):
        chunk = bytes(self.received[:length])
        del self.received[:length]
        return chunk

    def write(self, data):
        self.written.append(bytes(data))
        return len(data)


class ClockedTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        clock = patch.object(web.time, "monotonic", side_effect=lambda: self.now)
        clock.start()
        self.addCleanup(clock.stop)


class SharedStateTests(ClockedTest):
    def setUp(self):
        super().setUp()
        self.state = web.SharedControlState(0.35)
        self.client = "small-yaw-test-client"
        self.state.claim(self.client)

    def update(self, sequence=1, armed=False, keys=None, yaw=False, spin=False, scale=1.0):
        return self.state.update(
            self.client, sequence, armed, keys or [], scale, spin, yaw
        )[2]

    def test_unknown_feedback_prevents_yaw_arm(self):
        snapshot = self.update(keys=["j"], yaw=True)
        self.assertIsNone(snapshot["small_yaw_online"])
        self.assertFalse(snapshot["small_yaw_armed"])
        self.assertEqual(snapshot["keys"], [])

    def test_independent_arm_filters_wrong_axis_keys(self):
        self.state.set_small_yaw_online(True)
        self.state.set_small_yaw_left_limit(False)
        self.state.set_small_yaw_right_limit(False)
        snapshot = self.update(keys=["j", "w"], yaw=True, spin=True)
        self.assertTrue(snapshot["small_yaw_armed"])
        self.assertFalse(snapshot["armed"])
        self.assertFalse(snapshot["spin_enabled"])
        self.assertEqual(snapshot["keys"], ["j"])
        snapshot = self.update(sequence=2, armed=True, keys=["w", "j"], spin=True)
        self.assertEqual(snapshot["keys"], ["w"])
        self.assertTrue(snapshot["spin_enabled"])

    def test_feedback_timeout_disarms_only_yaw(self):
        self.state.set_small_yaw_online(True)
        self.update(armed=True, keys=["j", "w"], yaw=True, spin=True)
        self.now += 0.26
        snapshot = self.state.snapshot()
        self.assertIsNone(snapshot["small_yaw_online"])
        self.assertFalse(snapshot["small_yaw_armed"])
        self.assertTrue(snapshot["armed"])
        self.assertTrue(snapshot["spin_enabled"])
        self.assertEqual(snapshot["keys"], ["w"])

    def test_offline_feedback_disarms_without_waiting(self):
        self.state.set_small_yaw_online(True)
        self.update(keys=["j"], yaw=True)
        self.state.set_small_yaw_online(False)
        snapshot = self.state.snapshot()
        self.assertFalse(snapshot["small_yaw_online"])
        self.assertFalse(snapshot["small_yaw_armed"])
        self.assertEqual(snapshot["keys"], [])

    def test_yaw_only_watchdog_releases_ownership(self):
        self.state.set_small_yaw_online(True)
        self.update(keys=["j"], yaw=True)
        self.now += 0.36
        self.state.set_small_yaw_online(True)
        snapshot = self.state.snapshot()
        self.assertIsNone(snapshot["active_client"])
        self.assertFalse(snapshot["small_yaw_armed"])
        self.assertEqual(snapshot["keys"], [])

    def test_claim_and_release_clear_both_axes(self):
        self.state.set_small_yaw_online(True)
        self.update(armed=True, keys=["j", "w"], yaw=True, spin=True)
        snapshot = self.state.claim("different-test-client")
        for field in ("armed", "small_yaw_armed", "spin_enabled"):
            self.assertFalse(snapshot[field])
        self.assertEqual(snapshot["keys"], [])
        accepted, _, _ = self.state.update(self.client, 2, True, ["w"], 1.0)
        self.assertFalse(accepted)
        snapshot = self.state.release("different-test-client")
        self.assertIsNone(snapshot["active_client"])

    def test_stale_sequence_cannot_restore_released_keys(self):
        self.state.set_small_yaw_online(True)
        self.update(sequence=1, keys=["j"], yaw=True)
        self.update(sequence=2, keys=[], yaw=False)
        snapshot = self.update(sequence=1, keys=["j"], yaw=True)
        self.assertFalse(snapshot["small_yaw_armed"])
        self.assertEqual(snapshot["keys"], [])

    def test_nonfinite_scale_rejected(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.update(scale=value)

    def test_right_limit_blocks_right_but_allows_left(self):
        self.state.set_small_yaw_online(True)
        self.state.set_small_yaw_left_limit(False)
        self.state.set_small_yaw_right_limit(True)
        self.assertEqual(self.update(keys=["l"], yaw=True)["keys"], [])
        self.assertEqual(self.update(sequence=2, keys=["j"], yaw=True)["keys"], ["j"])
        self.assertEqual(self.update(sequence=3, keys=["j", "l"], yaw=True)["keys"], [])

    def test_unknown_or_stale_limit_feedback_blocks_right(self):
        self.state.set_small_yaw_online(True)
        self.assertEqual(self.update(keys=["l"], yaw=True)["keys"], [])
        self.state.set_small_yaw_right_limit(False)
        self.assertEqual(self.update(sequence=2, keys=["l"], yaw=True)["keys"], ["l"])
        self.now += 0.26
        self.state.set_small_yaw_online(True)
        snapshot = self.state.snapshot()
        self.assertIsNone(snapshot["small_yaw_right_limit"])
        self.assertEqual(snapshot["keys"], [])

    def test_left_limit_blocks_left_but_allows_right(self):
        self.state.set_small_yaw_online(True)
        self.state.set_small_yaw_left_limit(True)
        self.state.set_small_yaw_right_limit(False)
        self.assertEqual(self.update(keys=["j"], yaw=True)["keys"], [])
        self.assertEqual(self.update(sequence=2, keys=["l"], yaw=True)["keys"], ["l"])
        self.assertEqual(self.update(sequence=3, keys=["j", "l"], yaw=True)["keys"], [])

    def test_unknown_and_stale_left_feedback_do_not_block_escape_right(self):
        self.state.set_small_yaw_online(True)
        self.state.set_small_yaw_right_limit(False)
        self.assertEqual(self.update(keys=["j"], yaw=True)["keys"], [])
        self.assertEqual(self.update(sequence=2, keys=["l"], yaw=True)["keys"], ["l"])
        self.state.set_small_yaw_left_limit(False)
        self.assertEqual(self.update(sequence=3, keys=["j"], yaw=True)["keys"], ["j"])
        self.now += 0.26
        self.state.set_small_yaw_online(True)
        self.state.set_small_yaw_right_limit(False)
        snapshot = self.state.snapshot()
        self.assertIsNone(snapshot["small_yaw_left_limit"])
        self.assertEqual(snapshot["keys"], [])
        self.assertEqual(self.update(sequence=4, keys=["l"], yaw=True)["keys"], ["l"])

    def test_live_limit_discards_held_key_without_resuming_it(self):
        self.state.set_small_yaw_online(True)
        self.state.set_small_yaw_left_limit(False)
        self.state.set_small_yaw_right_limit(False)
        self.assertEqual(self.update(keys=["j"], yaw=True)["keys"], ["j"])
        self.state.set_small_yaw_left_limit(True)
        self.assertEqual(self.state.snapshot()["keys"], [])
        self.state.set_small_yaw_left_limit(False)
        self.assertEqual(self.state.snapshot()["keys"], [])

    def test_diagnostics_update_limits_and_expire_together(self):
        self.state.set_small_yaw_feedback([4569, -120.4, -120.4, 0.0, 0, 3, 1234])
        snapshot = self.state.snapshot()
        self.assertTrue(snapshot["small_yaw_online"])
        self.assertTrue(snapshot["small_yaw_left_limit"])
        self.assertFalse(snapshot["small_yaw_right_limit"])
        self.assertEqual(snapshot["small_yaw_ecd"], 4569)
        self.assertAlmostEqual(snapshot["small_yaw_error_deg"], 0.0)
        self.assertEqual(self.update(keys=["j"], yaw=True)["keys"], [])
        self.assertEqual(self.update(sequence=2, keys=["l"], yaw=True)["keys"], ["l"])
        self.now += 0.26
        snapshot = self.state.snapshot()
        for field in ("small_yaw_online", "small_yaw_left_limit", "small_yaw_right_limit",
                      "small_yaw_ecd", "small_yaw_actual_deg", "small_yaw_target_deg",
                      "small_yaw_speed_deg_s", "small_yaw_output"):
            self.assertIsNone(snapshot[field], field)
        self.assertFalse(snapshot["small_yaw_armed"])
        self.assertEqual(snapshot["keys"], [])

    def test_chassis_readiness_reports_each_motor_group_and_control_enable(self):
        cases = (
            (0x71, True, True, True, False),
            (0x31, False, True, False, False),
            (0x51, False, False, True, False),
            (0xF1, True, True, True, True),
            (0x11, False, False, False, False),
        )
        for flags, ready, steer, drive, enabled in cases:
            with self.subTest(flags=hex(flags)):
                self.assertTrue(self.state.set_small_yaw_feedback(
                    [5491, -118.7, -118.7, 0.0, 0, flags, 1234]
                ))
                snapshot = self.state.snapshot()
                self.assertEqual(snapshot["chassis_ready"], ready)
                self.assertEqual(snapshot["chassis_steer_online"], steer)
                self.assertEqual(snapshot["chassis_drive_online"], drive)
                self.assertEqual(snapshot["chassis_control_enabled"], enabled)
                if flags == 0x31:
                    self.assertIn("驱动", snapshot["chassis_block_reason"])
                elif flags == 0x51:
                    self.assertIn("舵向", snapshot["chassis_block_reason"])

    def test_old_firmware_and_expired_feedback_never_claim_chassis_ready(self):
        self.assertTrue(self.state.set_small_yaw_feedback(
            [5491, -118.7, -118.7, 0.0, 0, 0x0F, 1234]
        ))
        fields = ("chassis_ready", "chassis_steer_online", "chassis_drive_online", "chassis_control_enabled")
        for field in fields:
            self.assertIsNone(self.state.snapshot()[field], field)
        self.assertTrue(self.state.set_small_yaw_feedback(
            [5491, -118.7, -118.7, 0.0, 0, 0xF1, 1234]
        ))
        self.assertTrue(self.state.snapshot()["chassis_ready"])
        self.now += 0.26
        for field in fields:
            self.assertIsNone(self.state.snapshot()[field], field)

    def test_high_status_bits_without_capability_never_refresh_feedback(self):
        for flags in (0x20, 0x40, 0x80, 0xE1, 256, -1):
            with self.subTest(flags=flags):
                self.assertFalse(self.state.set_small_yaw_feedback(
                    [5491, -118.7, -118.7, 0.0, 0, flags, 1234]
                ))
                snapshot = self.state.snapshot()
                self.assertIsNone(snapshot["small_yaw_online"])
                self.assertIsNone(snapshot["chassis_ready"])

    def test_malformed_diagnostics_never_refresh_feedback(self):
        valid = [4569, -120.4, -120.3, 6.0, 300, 3, 1234]
        cases = [valid[:-1], valid + [0]]
        for index, value in ((0, 8192), (0, -1), (0, 4569.5),
                             (1, float("nan")), (2, float("inf")),
                             (3, float("nan")), (5, 0x20), (5, 1.5)):
            case = valid.copy()
            case[index] = value
            cases.append(case)
        for values in cases:
            with self.subTest(values=values):
                self.state.set_small_yaw_feedback(values)
                snapshot = self.state.snapshot()
                self.assertIsNone(snapshot["small_yaw_online"])
                self.assertIsNone(snapshot["small_yaw_left_limit"])
                self.assertIsNone(snapshot["small_yaw_right_limit"])


class PublisherTests(ClockedTest):
    def setUp(self):
        super().setUp()
        self.state = web.SharedControlState(0.35)
        self.state.claim("publisher-test-client")
        self.state.set_small_yaw_online(True)
        self.state.set_small_yaw_left_limit(False)
        self.state.set_small_yaw_right_limit(False)
        self.node = SimpleNamespace(
            control_state=self.state,
            _publisher=FakePublisher(),
            _manual_override_publisher=FakePublisher(),
            _manual_override=False,
            _gimbal_publisher=FakePublisher(),
            _linear_speed=0.8,
            _angular_speed=2.4,
            _was_armed=False,
            _zero_frames_remaining=0,
            _was_gimbal_armed=False,
            _gimbal_zero_frames_remaining=0,
        )
        self.node._set_manual_override = lambda active: web.BrowserTeleop._set_manual_override(
            self.node, active
        )
        self.sequence = 0

    def update(self, armed=False, keys=None, yaw=False, spin=False, scale=1.0):
        self.sequence += 1
        self.state.update(
            "publisher-test-client", self.sequence, armed, keys or [], scale, spin, yaw
        )
        web.BrowserTeleop._publish_tick(self.node)

    def test_yaw_only_rate_scaling_and_no_chassis_output(self):
        self.update(keys=["j", "w"], yaw=True, scale=0.4)
        self.assertEqual(self.node._publisher.messages, [])
        message = self.node._gimbal_publisher.messages[-1]
        self.assertAlmostEqual(message.x, 0.24)
        self.assertEqual((message.y, message.z), (0.0, 0.0))
        self.update(keys=["l"], yaw=True)
        self.assertAlmostEqual(self.node._gimbal_publisher.messages[-1].x, -0.60)

    def test_all_scales_raise_only_small_yaw_and_restore_chassis_rates(self):
        for scale in (0.2, 0.4, 0.6, 0.8, 1.0):
            for direction in (1, -1):
                with self.subTest(scale=scale, direction=direction):
                    keys = ["w", "q", "j"] if direction > 0 else ["s", "e", "l"]
                    self.update(armed=True, keys=keys, yaw=True, scale=scale)
                    twist = self.node._publisher.messages[-1]
                    gimbal = self.node._gimbal_publisher.messages[-1]
                    self.assertAlmostEqual(twist.linear.x, direction * 0.8 * scale)
                    self.assertEqual(twist.linear.y, 0.0)
                    self.assertAlmostEqual(twist.angular.z, direction * 2.4 * scale)
                    self.assertAlmostEqual(gimbal.x, direction * 0.60 * scale)
                    self.assertEqual((gimbal.y, gimbal.z), (0.0, 0.0))
            with self.subTest(scale=scale, axis="lateral"):
                self.update(armed=True, keys=["a"], scale=scale)
                self.assertAlmostEqual(self.node._publisher.messages[-1].linear.y, 0.8 * scale)
                self.update(armed=True, keys=["d"], scale=scale)
                self.assertAlmostEqual(self.node._publisher.messages[-1].linear.y, -0.8 * scale)

    def test_beyblade_rate_is_restored_and_independent_of_speed_scale(self):
        for scale in (0.2, 0.4, 0.6, 0.8, 1.0):
            with self.subTest(scale=scale):
                self.update(armed=True, keys=["w", "q", "j"], yaw=True, spin=True, scale=scale)
                self.assertAlmostEqual(self.node._publisher.messages[-1].linear.x, 0.8 * scale)
                self.assertEqual(self.node._publisher.messages[-1].angular.z, 0.5)
                self.assertAlmostEqual(self.node._gimbal_publisher.messages[-1].x, 0.60 * scale)
                self.assertEqual(self.node._gimbal_publisher.messages[-1].z, 1.0)

    def test_browser_defaults_raise_yaw_without_raising_chassis(self):
        with patch.object(sys, "argv", ["wasd_web_teleop.py"]):
            arguments, _ = web.parse_arguments()
        self.assertEqual((arguments.speed, arguments.turn), (0.8, 2.4))
        limits = web.BrowserTeleop.limits.fget(self.node)
        self.assertEqual((limits["linear_speed"], limits["angular_speed"]), (0.8, 2.4))
        self.assertEqual((limits["small_yaw_rate"], limits["beyblade_spin_rate"]), (0.60, 0.5))

    def test_opposite_yaw_keys_cancel_and_release_sends_zero(self):
        self.update(keys=["j", "l"], yaw=True)
        self.assertEqual(self.node._gimbal_publisher.messages[-1].x, 0.0)
        self.update(keys=["j"], yaw=True)
        self.update(keys=[], yaw=True)
        self.assertEqual(self.node._gimbal_publisher.messages[-1].x, 0.0)

    def test_beyblade_and_translation_preserved_with_yaw(self):
        self.update(armed=True, keys=["w", "a", "j"], yaw=True, spin=True)
        twist = self.node._publisher.messages[-1]
        self.assertAlmostEqual((twist.linear.x ** 2 + twist.linear.y ** 2) ** 0.5, 0.8)
        self.assertEqual(twist.angular.z, web.BEYBLADE_SPIN_RATE_RAD_S)
        gimbal = self.node._gimbal_publisher.messages[-1]
        self.assertAlmostEqual(gimbal.x, 0.60)
        self.assertEqual(gimbal.z, 1.0)

    def test_yaw_disarm_emits_four_zeros_without_chassis_output(self):
        self.update(keys=["j"], yaw=True)
        self.update()
        for _ in range(5):
            web.BrowserTeleop._publish_tick(self.node)
        messages = self.node._gimbal_publisher.messages
        self.assertEqual(len(messages), 1 + web.ZERO_BURST_FRAMES)
        self.assertTrue(all(message.x == message.y == message.z == 0.0 for message in messages[1:]))
        self.assertEqual(self.node._publisher.messages, [])

    def test_chassis_disarm_zeros_do_not_interrupt_yaw(self):
        self.update(armed=True, keys=["w", "j"], yaw=True)
        self.update(keys=["j"], yaw=True)
        for _ in range(5):
            web.BrowserTeleop._publish_tick(self.node)
        self.assertEqual(len(self.node._publisher.messages), 1 + web.ZERO_BURST_FRAMES)
        self.assertTrue(all(message.linear.x == 0 for message in self.node._publisher.messages[1:]))
        self.assertTrue(all(message.x == 0.60 for message in self.node._gimbal_publisher.messages))

    def test_yaw_only_shutdown_does_not_claim_chassis(self):
        self.update(keys=["j"], yaw=True)
        with patch.object(web.rclpy, "ok", return_value=True), patch.object(web.time, "sleep"):
            web.BrowserTeleop.publish_shutdown_stop(self.node)
        self.assertEqual(self.node._publisher.messages, [])
        self.assertEqual(len(self.node._gimbal_publisher.messages), 1 + web.ZERO_BURST_FRAMES)

    def test_manual_override_only_on_motion_and_persists_until_release(self):
        self.update(keys=["j"], yaw=True)
        self.update(armed=True)
        self.assertEqual(self.node._manual_override_publisher.messages, [])

        self.update(armed=True, keys=["w"])
        self.update(armed=True, keys=["d"])
        self.update(armed=True)
        self.assertEqual(
            [message.data for message in self.node._manual_override_publisher.messages], [True]
        )

        self.update()
        self.assertEqual(
            [message.data for message in self.node._manual_override_publisher.messages],
            [True, False],
        )

    def test_manual_override_releases_on_watchdog_or_shutdown(self):
        self.update(armed=True, spin=True)
        self.now += 0.36
        web.BrowserTeleop._publish_tick(self.node)
        self.assertFalse(self.node._manual_override_publisher.messages[-1].data)

        self.state.claim("publisher-test-client")
        self.update(armed=True, keys=["w"])
        with patch.object(web.rclpy, "ok", return_value=True), patch.object(web.time, "sleep"):
            web.BrowserTeleop.publish_shutdown_stop(self.node)
        self.assertEqual(
            [message.data for message in self.node._manual_override_publisher.messages],
            [True, False, True, False],
        )


class BridgeTests(ClockedTest):
    def telemetry_node(self, received=b""):
        node = SimpleNamespace(
            _serial=FakeSerial(received),
            _rx_buffer=bytearray(),
            _rx_bytes_total=0,
            _valid_frame_count=0,
            _crc_error_count=0,
            _discarded_byte_count=0,
            _last_wrong_firmware_warning=0.0,
            _last_motor_status=None,
            _aim_yaw_online_publisher=FakePublisher(),
            _aim_yaw_left_limit_publisher=FakePublisher(),
            _aim_yaw_right_limit_publisher=FakePublisher(),
            _aim_yaw_feedback_publisher=FakePublisher(),
            get_logger=lambda: Mock(),
            _handle_telemetry=Mock(),
            _disconnect_serial=Mock(side_effect=AssertionError("unexpected serial failure")),
        )
        node._handle_aim_telemetry = lambda fields: bridge.SentryNav2Bridge._handle_aim_telemetry(node, fields)
        return node

    def aim_telemetry(self, sequence=1, ecd=4569, actual=-120.4,
                      target=-120.3, speed=0.0, output=0, flags=3):
        body = bridge.AIM_TELEMETRY_BODY.pack(
            bridge.MAGIC, bridge.PROTOCOL_VERSION, bridge.AIM_TELEMETRY_TYPE,
            sequence, 1234, ecd, actual, target, speed, output, flags,
        )
        return body + bridge.CRC.pack(bridge.crc16_ccitt(body))

    def command_node(self):
        parameters = {
            "command_timeout_s": 0.25,
            "max_translation_m_s": 0.8,
            "max_angular_rad_s": 2.4,
        }
        node = SimpleNamespace(
            _serial=FakeSerial(),
            _last_twist_monotonic=None,
            _latest_twist=bridge.Twist(),
            _emergency_stop=False,
            _last_gimbal_monotonic=self.now,
            _latest_gimbal=bridge.Vector3(x=0.15),
            _command_sequence=0,
            get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
            _disconnect_serial=Mock(side_effect=AssertionError("unexpected serial failure")),
        )
        node._send_command = lambda **kwargs: bridge.SentryNav2Bridge._send_command(node, **kwargs)
        return node

    def unpack_command(self, frame):
        self.assertEqual(len(frame), bridge.COMMAND_FRAME_SIZE)
        self.assertEqual(bridge.crc16_ccitt(frame[:-2]), bridge.CRC.unpack(frame[-2:])[0])
        return bridge.COMMAND_BODY.unpack(frame[:-2])

    def test_yaw_rate_is_150_mrad_in_dedicated_gimbal_frame(self):
        node = self.command_node()
        bridge.SentryNav2Bridge._send_command(node)
        chassis, yaw = [self.unpack_command(frame) for frame in node._serial.written]
        self.assertEqual(chassis[2], bridge.COMMAND_TYPE)
        self.assertEqual(chassis[4:], (0, 0, 0, 0))
        self.assertEqual(yaw[2], bridge.GIMBAL_COMMAND_TYPE)
        self.assertEqual(yaw[4:], (150, 0, 0, bridge.COMMAND_ENABLE))

    def test_higher_yaw_rate_reaches_wire_and_bounds_excess(self):
        for requested, expected in ((0.60, 600), (-0.60, -600), (1.20, 600), (-1.20, -600)):
            with self.subTest(requested=requested):
                node = self.command_node()
                node._latest_gimbal.x = requested
                bridge.SentryNav2Bridge._send_command(node)
                self.assertEqual(self.unpack_command(node._serial.written[-1])[4:],
                                 (expected, 0, 0, bridge.COMMAND_ENABLE))

    def test_yaw_speed_increase_preserves_pitch_wire_limit(self):
        for direction in (-1, 1):
            node = self.command_node()
            node._latest_gimbal.x = direction * 1.2
            node._latest_gimbal.y = direction * 1.2
            bridge.SentryNav2Bridge._send_command(node)
            self.assertEqual(self.unpack_command(node._serial.written[-1])[4:],
                             (direction * 600, direction * 200, 0, bridge.COMMAND_ENABLE))

    def test_standalone_chassis_caps_restore_original_rates_and_bound_excess(self):
        for requested, expected in ((0.8, 800), (-0.8, -800), (1.6, 800), (-1.6, -800)):
            for angular, angular_expected in ((2.4, 2400), (-2.4, -2400), (4.8, 2400), (-4.8, -2400)):
                with self.subTest(requested=requested, angular=angular):
                    node = self.command_node()
                    node._last_twist_monotonic = self.now
                    node._latest_twist.linear.x = requested
                    node._latest_twist.angular.z = angular
                    bridge.SentryNav2Bridge._send_command(node)
                    self.assertEqual(self.unpack_command(node._serial.written[0])[4:],
                                     (expected, 0, angular_expected, bridge.COMMAND_ENABLE))

    def test_standalone_diagonal_speed_is_limited_as_a_vector(self):
        node = self.command_node()
        node._last_twist_monotonic = self.now
        node._latest_twist.linear.x = 0.8
        node._latest_twist.linear.y = 0.8
        bridge.SentryNav2Bridge._send_command(node)
        frame = self.unpack_command(node._serial.written[0])
        self.assertEqual(frame[4:6], (566, 566))
        self.assertLessEqual(math.hypot(*frame[4:6]), 801)

    def test_emergency_stop_immediately_revokes_chassis_only(self):
        node = self.command_node()
        motion = bridge.Twist()
        motion.linear.x = 0.4
        receive_twist = bridge.SentryNav2Bridge._twist_callback
        receive_stop = bridge.SentryNav2Bridge._emergency_stop_callback

        receive_twist(node, motion)
        node._send_command()
        self.assertEqual(self.unpack_command(node._serial.written[0])[4:],
                         (400, 0, 0, bridge.COMMAND_ENABLE))

        node._serial.written.clear()
        receive_stop(node, bridge.Bool(data=True))
        self.assertEqual(len(node._serial.written), 1)
        immediate = self.unpack_command(node._serial.written[0])
        self.assertEqual(immediate[2], bridge.COMMAND_TYPE)
        self.assertEqual(immediate[4:], (0, 0, 0, 0))

        receive_twist(node, motion)
        receive_stop(node, bridge.Bool(data=False))
        node._send_command()
        self.assertEqual(self.unpack_command(node._serial.written[-2])[4:], (0, 0, 0, 0))
        self.assertEqual(self.unpack_command(node._serial.written[-1])[4:],
                         (150, 0, 0, bridge.COMMAND_ENABLE))

        receive_twist(node, motion)
        node._send_command()
        self.assertEqual(self.unpack_command(node._serial.written[-2])[4:],
                         (400, 0, 0, bridge.COMMAND_ENABLE))

    def test_stale_gimbal_command_disables_output_and_spin(self):
        node = self.command_node()
        node._latest_gimbal.z = 1.0
        self.now += 0.26
        bridge.SentryNav2Bridge._send_command(node)
        self.assertEqual(self.unpack_command(node._serial.written[-1])[4:], (0, 0, 0, 0))

    def test_beyblade_flag_can_share_yaw_command(self):
        node = self.command_node()
        node._latest_gimbal.z = 1.0
        bridge.SentryNav2Bridge._send_command(node)
        self.assertEqual(self.unpack_command(node._serial.written[-1])[4:], (150, 0, 0, 3))

    def test_corrupt_telemetry_never_publishes_online_status(self):
        def telemetry(status, sequence):
            body = bridge.TELEMETRY_BODY.pack(
                bridge.MAGIC, bridge.PROTOCOL_VERSION, bridge.TELEMETRY_TYPE,
                sequence, 1234, 1, 2, 3, 4, 0, 0, 0, 0, status,
            )
            return body + bridge.CRC.pack(bridge.crc16_ccitt(body))

        corrupt = bytearray(telemetry(0x44, 1))
        corrupt[15] ^= 0x80
        received = (
            bytes(corrupt) + telemetry(0x04, 2) + telemetry(0x44, 3)
            + telemetry(0x44, 4) + telemetry(0xC4, 5) + telemetry(0x40, 6)
        )
        node = self.telemetry_node(received)
        bridge.SentryNav2Bridge._read_telemetry(node)
        self.assertEqual(node._valid_frame_count, 5)
        self.assertGreaterEqual(node._crc_error_count, 1)
        self.assertEqual([message.data for message in node._aim_yaw_online_publisher.messages], [True, True, True, False])
        self.assertEqual(node._aim_yaw_right_limit_publisher.messages, [])
        self.assertEqual(node._aim_yaw_left_limit_publisher.messages, [])
        self.assertEqual(node._handle_telemetry.call_count, 5)

    def test_aim_frame_reports_both_limits_and_diagnostics(self):
        node = self.telemetry_node(
            self.aim_telemetry(flags=3) + self.aim_telemetry(sequence=2, ecd=7303, flags=5)
        )
        bridge.SentryNav2Bridge._read_telemetry(node)
        self.assertEqual([message.data for message in node._aim_yaw_online_publisher.messages], [True, True])
        self.assertEqual([message.data for message in node._aim_yaw_left_limit_publisher.messages], [True, False])
        self.assertEqual([message.data for message in node._aim_yaw_right_limit_publisher.messages], [False, True])
        values = node._aim_yaw_feedback_publisher.messages[0].data
        self.assertEqual(len(values), 7)
        self.assertEqual(values[0], 4569)
        self.assertAlmostEqual(values[1], -120.4, places=4)
        self.assertAlmostEqual(values[2], -120.3, places=4)
        self.assertEqual(list(values[3:]), [0.0, 0.0, 3.0, 1234.0])
        node._handle_telemetry.assert_not_called()

    def test_aim_frame_preserves_new_readiness_flags_for_browser(self):
        node = self.telemetry_node(self.aim_telemetry(ecd=5491, flags=0xF1))
        bridge.SentryNav2Bridge._read_telemetry(node)
        self.assertEqual(node._valid_frame_count, 1)
        self.assertEqual(node._aim_yaw_feedback_publisher.messages[-1].data[5], 0xF1)
        self.assertEqual([message.data for message in node._aim_yaw_online_publisher.messages], [True])
        self.assertEqual([message.data for message in node._aim_yaw_left_limit_publisher.messages], [False])
        self.assertEqual([message.data for message in node._aim_yaw_right_limit_publisher.messages], [False])

    def test_aim_frame_rejects_readiness_bits_without_capability(self):
        node = self.telemetry_node(b"".join(self.aim_telemetry(flags=flags) for flags in (0x20, 0x40, 0x80, 0xE1)))
        bridge.SentryNav2Bridge._read_telemetry(node)
        self.assertEqual(node._valid_frame_count, 0)
        self.assertEqual(node._aim_yaw_feedback_publisher.messages, [])
        self.assertEqual(node._aim_yaw_online_publisher.messages, [])

    def test_bad_aim_crc_and_invalid_values_never_refresh_status(self):
        corrupt = bytearray(self.aim_telemetry())
        corrupt[13] ^= 0x80
        invalid = b"".join(self.aim_telemetry(**case) for case in (
            {"ecd": 8192}, {"actual": float("nan")}, {"target": float("inf")},
            {"speed": float("nan")}, {"flags": 0x20},
        ))
        node = self.telemetry_node(bytes(corrupt) + invalid)
        bridge.SentryNav2Bridge._read_telemetry(node)
        self.assertGreaterEqual(node._crc_error_count, 1)
        for name in ("_aim_yaw_online_publisher", "_aim_yaw_left_limit_publisher",
                     "_aim_yaw_right_limit_publisher", "_aim_yaw_feedback_publisher"):
            self.assertEqual(getattr(node, name).messages, [], name)

    def test_split_and_bundled_frames_preserve_chassis_and_aim(self):
        body = bridge.TELEMETRY_BODY.pack(
            bridge.MAGIC, bridge.PROTOCOL_VERSION, bridge.TELEMETRY_TYPE,
            1, 1234, 1, 2, 3, 4, 0, 0, 0, 0, 0x44,
        )
        chassis = body + bridge.CRC.pack(bridge.crc16_ccitt(body))
        aim = self.aim_telemetry()
        node = self.telemetry_node(chassis + aim[:11])
        bridge.SentryNav2Bridge._read_telemetry(node)
        self.assertEqual(node._handle_telemetry.call_count, 1)
        self.assertEqual(node._aim_yaw_left_limit_publisher.messages, [])
        node._serial.received.extend(aim[11:])
        bridge.SentryNav2Bridge._read_telemetry(node)
        self.assertEqual([message.data for message in node._aim_yaw_left_limit_publisher.messages], [True])
        self.assertEqual(node._handle_telemetry.call_count, 1)


class FirmwareLimitTests(unittest.TestCase):
    """Exercise the production C helper, without copying its rules to Python."""

    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("cc")
        if compiler is None:
            raise unittest.SkipTest("Host C compiler is unavailable")
        cls.directory = tempfile.TemporaryDirectory(prefix="small_yaw_limit_test_")
        cls.addClassCleanup(cls.directory.cleanup)
        library = Path(cls.directory.name) / "yaw_limit.so"
        source = r'''
#include "sentry_aim_yaw_limit.h"
#include "sentry_aim_yaw_profile.h"
float relative(unsigned short ecd, unsigned short right) {
    return SentryYawRelativeDeg(ecd, right);
}
float step(unsigned short ecd, unsigned short right, float total, float previous,
           float rate, signed char sign, float left, float guard, float dt,
           unsigned char active, unsigned char *enabled, unsigned char *at_right,
           unsigned char *at_left) {
    SentryYawLimitResult_s result = SentryYawLimitStep(
        ecd, right, total, previous, rate, sign, left, guard, dt, active);
    *enabled = result.enabled;
    *at_right = result.at_right_limit;
    *at_left = result.at_left_limit;
    return result.target_deg;
}
float clamp_output(float output, float relative, float left, float guard) {
    return SentryYawClampOutput(output, relative, left, guard);
}
unsigned char limit_flags(float relative, float left, float guard,
                          float engage, float release,
                          unsigned char previous_left, unsigned char previous_right) {
    SentryYawLimitFlags_s previous = {
        .at_left_limit = previous_left,
        .at_right_limit = previous_right,
    };
    SentryYawLimitFlags_s result = SentryYawUpdateLimitFlags(
        relative, left, guard, engage, release, previous);
    return (result.at_left_limit ? 1u : 0u) | (result.at_right_limit ? 2u : 0u);
}
float profile(float requested, float previous, float relative, float target_relative,
              float left, float guard, float max_rate, float accel, float brake, float dt,
              unsigned char at_left, unsigned char at_right, unsigned char *valid) {
    SentryYawProfileResult_s result = SentryYawProfileStep(
        requested, previous, relative, target_relative, left, guard,
        max_rate, accel, brake, dt, at_left, at_right);
    *valid = result.valid;
    return result.rate_deg_s;
}
'''
        result = subprocess.run(
            [compiler, "-std=c11", "-shared", "-fPIC", "-Wall", "-Wextra", "-Werror",
             "-I", str(ROOT / "application/sentry"), "-x", "c", "-", "-o", str(library), "-lm"],
            input=source, text=True, capture_output=True, check=False,
        )
        if result.returncode:
            raise AssertionError(result.stderr)
        cls.library = ctypes.CDLL(str(library))
        cls.library.relative.argtypes = [ctypes.c_uint16, ctypes.c_uint16]
        cls.library.relative.restype = ctypes.c_float
        cls.library.step.argtypes = [
            ctypes.c_uint16, ctypes.c_uint16, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_int8, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_uint8),
        ]
        cls.library.step.restype = ctypes.c_float
        cls.library.clamp_output.argtypes = [ctypes.c_float] * 4
        cls.library.clamp_output.restype = ctypes.c_float
        cls.library.limit_flags.argtypes = [ctypes.c_float] * 5 + [ctypes.c_uint8] * 2
        cls.library.limit_flags.restype = ctypes.c_uint8
        cls.library.profile.argtypes = [ctypes.c_float] * 10 + [ctypes.c_uint8] * 2 + [
            ctypes.POINTER(ctypes.c_uint8)
        ]
        cls.library.profile.restype = ctypes.c_float
        cls.guard = 4 * 360 / 8192

    def step(self, ecd=7309, right=7309, total=1000.0, previous=1000.0,
             rate=150.0, sign=-1, left=10.0, guard=None, dt=0.005, active=0):
        enabled = ctypes.c_uint8()
        at_right = ctypes.c_uint8()
        at_left = ctypes.c_uint8()
        target = self.library.step(
            ecd, right, total, previous, rate, sign, left,
            self.guard if guard is None else guard, dt, active,
            ctypes.byref(enabled), ctypes.byref(at_right), ctypes.byref(at_left),
        )
        return target, bool(enabled.value), bool(at_right.value), bool(at_left.value)

    def limit_flags(self, ecd=4590, right=7309, previous=(False, False), **overrides):
        degrees_per_count = 360 / 8192
        parameters = {
            "relative": self.library.relative(ecd, right),
            "left": (2740 - 16) * degrees_per_count,
            "guard": 16 * degrees_per_count,
            "engage": 8 * degrees_per_count,
            "release": 32 * degrees_per_count,
        }
        parameters.update(overrides)
        flags = self.library.limit_flags(
            *(parameters[name] for name in ("relative", "left", "guard", "engage", "release")),
            *previous,
        )
        return bool(flags & 1), bool(flags & 2)

    def profile(self, **overrides):
        parameters = {
            "requested": math.degrees(0.6), "previous": 0.0,
            "relative": -60.0, "target_relative": -60.0,
            "left": (2740 - 16) * 360 / 8192, "guard": 16 * 360 / 8192,
            "max_rate": math.degrees(0.6), "accel": math.degrees(2.0),
            "brake": math.degrees(3.0), "dt": 0.005,
            "at_left": False, "at_right": False,
        }
        parameters.update(overrides)
        valid = ctypes.c_uint8()
        rate = self.library.profile(
            *(parameters[name] for name in (
                "requested", "previous", "relative", "target_relative", "left", "guard",
                "max_rate", "accel", "brake", "dt", "at_left", "at_right",
            )),
            ctypes.byref(valid),
        )
        return rate, bool(valid.value)

    def test_profile_accelerates_to_new_max_rate_without_exceeding_it(self):
        max_rate = math.degrees(0.6)
        increment = math.degrees(2.0) * 0.005
        for direction in (-1, 1):
            previous = 0.0
            for _ in range(65):
                rate, valid = self.profile(requested=direction * max_rate * 2,
                                           previous=previous)
                self.assertTrue(valid)
                self.assertGreaterEqual(direction * rate, 0)
                self.assertLessEqual(abs(rate), max_rate + 0.0001)
                self.assertLessEqual(abs(rate - previous), increment + 0.0001)
                previous = rate
            self.assertAlmostEqual(previous, direction * max_rate, places=4)

    def test_profile_release_and_blocked_directions_zero_immediately(self):
        max_rate = math.degrees(0.6)
        for previous in (-max_rate, max_rate):
            self.assertEqual(self.profile(requested=0, previous=previous), (0.0, True))
        self.assertEqual(self.profile(requested=-max_rate, at_left=True), (0.0, True))
        self.assertEqual(self.profile(requested=max_rate, at_right=True), (0.0, True))
        self.assertEqual(self.profile(at_left=True, at_right=True), (0.0, True))

    def test_profile_reversal_and_speed_reduction_use_braking_acceleration(self):
        max_rate = math.degrees(0.6)
        step = math.degrees(3.0) * 0.005
        for direction in (-1, 1):
            rate, valid = self.profile(requested=-direction * max_rate,
                                       previous=direction * max_rate)
            self.assertTrue(valid)
            self.assertAlmostEqual(rate, direction * (max_rate - step), places=4)
            rate, valid = self.profile(requested=direction * max_rate / 2,
                                       previous=direction * max_rate)
            self.assertTrue(valid)
            self.assertAlmostEqual(rate, direction * (max_rate - step), places=4)

    def test_profile_brakes_for_closer_actual_or_target_boundary(self):
        max_rate = math.degrees(0.6)
        guard = 16 * 360 / 8192
        left = (2740 - 16) * 360 / 8192
        cap = math.sqrt(2 * math.degrees(3.0) * 0.1)
        for close_position in ("relative", "target_relative"):
            rate, valid = self.profile(previous=max_rate, **{close_position: -guard - 0.1})
            self.assertTrue(valid)
            self.assertGreater(rate, 0)
            self.assertAlmostEqual(rate, cap, places=3)
            rate, valid = self.profile(requested=-max_rate, previous=-max_rate,
                                       **{close_position: -left + 0.1})
            self.assertTrue(valid)
            self.assertLess(rate, 0)
            self.assertAlmostEqual(rate, -cap, places=3)

    def test_profile_exact_soft_edges_stop_outward_without_latch(self):
        max_rate = math.degrees(0.6)
        guard = 16 * 360 / 8192
        left = (2740 - 16) * 360 / 8192
        for brake in (math.degrees(3.0), 3.0e38):
            self.assertEqual(self.profile(previous=max_rate, relative=-guard,
                                           target_relative=-guard, brake=brake), (0.0, True))
            self.assertEqual(self.profile(requested=-max_rate, previous=-max_rate,
                                           relative=-left, target_relative=-left,
                                           brake=brake), (0.0, True))

    def test_profile_inward_escape_discards_old_outward_rate(self):
        max_rate = math.degrees(0.6)
        increment = math.degrees(2.0) * 0.005
        left = (2740 - 16) * 360 / 8192
        guard = 16 * 360 / 8192
        rate, valid = self.profile(previous=-max_rate, relative=-left - 1,
                                   target_relative=-left - 1, at_left=True)
        self.assertTrue(valid)
        self.assertAlmostEqual(rate, increment, places=4)
        rate, valid = self.profile(requested=-max_rate, previous=max_rate,
                                   relative=-guard + 1, target_relative=-guard + 1,
                                   at_right=True)
        self.assertTrue(valid)
        self.assertAlmostEqual(rate, -increment, places=4)

    def test_profile_invalid_inputs_fail_closed(self):
        for name in ("requested", "previous", "relative", "target_relative", "left",
                     "guard", "max_rate", "accel", "brake", "dt"):
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(name=name, value=value):
                    self.assertEqual(self.profile(**{name: value}), (0.0, False))
        for case in ({"guard": -1}, {"left": 0}, {"left": 180}, {"max_rate": 0},
                     {"accel": 0}, {"brake": -1}, {"dt": 0}, {"dt": -0.001}, {"dt": 0.051}):
            with self.subTest(case=case):
                self.assertEqual(self.profile(**case), (0.0, False))

    def test_firmware_angle_target_accepts_new_yaw_rate_and_bounds_excess(self):
        guard = 16 * 360 / 8192
        safe_left_travel = (2740 - 16) * 360 / 8192
        for requested, expected in ((600, 600), (-600, -600), (1200, 600), (-1200, -600)):
            with self.subTest(requested=requested):
                target, enabled, at_right, at_left = self.step(
                    ecd=6000, rate=requested, left=safe_left_travel, guard=guard,
                )
                self.assertTrue(enabled)
                self.assertFalse(at_left or at_right)
                self.assertAlmostEqual(target, 1000 - math.degrees(expected * 0.001) * 0.005,
                                       places=4)

    def test_limit_status_detects_observed_five_count_settling_error_after_restart(self):
        # The motor stopped at 4590, five counts inside the target boundary.
        # A fresh controller must report this edge without first crossing 4585.
        self.assertEqual(self.limit_flags(ecd=4590), (True, False))
        self.assertEqual(self.limit_flags(ecd=4593), (True, False))
        self.assertEqual(self.limit_flags(ecd=4594), (False, False))
        self.assertEqual(self.limit_flags(ecd=7285), (False, True))
        self.assertEqual(self.limit_flags(ecd=7284), (False, False))
        self.assertEqual(self.limit_flags(ecd=6000), (False, False))

    def test_left_limit_status_holds_through_backlash_until_clear_inward_escape(self):
        previous = self.limit_flags(ecd=4569)
        self.assertEqual(previous, (True, False))
        for ecd in (4590, 4594, 4592, 4600, 4595, 4616):
            with self.subTest(ecd=ecd):
                previous = self.limit_flags(ecd=ecd, previous=previous)
                self.assertEqual(previous, (True, False))
        previous = self.limit_flags(ecd=4617, previous=previous)
        self.assertEqual(previous, (False, False))
        # Entering the hysteresis band from the interior must not re-latch.
        previous = self.limit_flags(ecd=4600, previous=previous)
        self.assertEqual(previous, (False, False))
        self.assertEqual(self.limit_flags(ecd=4593, previous=previous), (True, False))

    def test_right_limit_status_holds_through_backlash_until_clear_inward_escape(self):
        previous = self.limit_flags(ecd=7309)
        self.assertEqual(previous, (False, True))
        for ecd in (7293, 7284, 7286, 7270, 7280, 7262):
            with self.subTest(ecd=ecd):
                previous = self.limit_flags(ecd=ecd, previous=previous)
                self.assertEqual(previous, (False, True))
        previous = self.limit_flags(ecd=7261, previous=previous)
        self.assertEqual(previous, (False, False))
        previous = self.limit_flags(ecd=7270, previous=previous)
        self.assertEqual(previous, (False, False))
        self.assertEqual(self.limit_flags(ecd=7285, previous=previous), (False, True))

    def test_limit_hysteresis_is_independent_of_encoder_zero(self):
        for right in (10, 1000, 8191):
            soft_left = (right - 2740 + 16) % 8192
            soft_right = (right - 16) % 8192
            with self.subTest(right=right):
                self.assertEqual(
                    self.limit_flags(ecd=(soft_left + 5) % 8192, right=right),
                    (True, False),
                )
                self.assertEqual(
                    self.limit_flags(ecd=(soft_left + 31) % 8192, right=right,
                                     previous=(True, False)),
                    (True, False),
                )
                self.assertEqual(
                    self.limit_flags(ecd=(soft_left + 32) % 8192, right=right,
                                     previous=(True, False)),
                    (False, False),
                )
                self.assertEqual(
                    self.limit_flags(ecd=(soft_right - 5) % 8192, right=right),
                    (False, True),
                )
                self.assertEqual(
                    self.limit_flags(ecd=(soft_right - 31) % 8192, right=right,
                                     previous=(False, True)),
                    (False, True),
                )
                self.assertEqual(
                    self.limit_flags(ecd=(soft_right - 32) % 8192, right=right,
                                     previous=(False, True)),
                    (False, False),
                )

    def test_invalid_limit_hysteresis_configuration_fails_closed(self):
        cases = (
            {"relative": float("nan")}, {"relative": float("inf")},
            {"left": 0}, {"left": 180}, {"left": float("nan")},
            {"guard": -1}, {"guard": float("inf")},
            {"engage": -1}, {"engage": float("nan")},
            {"release": 0}, {"release": 8 * 360 / 8192},
            {"release": float("nan")}, {"release": 60},
        )
        for case in cases:
            for previous in ((False, False), (True, False), (False, True)):
                with self.subTest(case=case, previous=previous):
                    self.assertEqual(self.limit_flags(previous=previous, **case), (True, True))

    def test_wrap_and_absolute_edge_do_not_recenter_after_restart(self):
        self.assertAlmostEqual(self.library.relative(1, 8191), 2 * 360 / 8192)
        self.assertAlmostEqual(self.library.relative(8191, 1), -2 * 360 / 8192)
        for total in (-720.0, 0.0, 1000.0):
            with self.subTest(total=total):
                target, enabled, at_right, _ = self.step(total=total, previous=total, rate=-150)
                self.assertFalse(enabled)
                self.assertTrue(at_right)
                self.assertEqual(target, total)

    def test_corrected_left_escapes_right_without_target_jump(self):
        for ecd in (7309, 7310, 7332):
            with self.subTest(ecd=ecd):
                target, enabled, at_right, _ = self.step(ecd=ecd, previous=2000, active=1)
                self.assertTrue(enabled)
                self.assertTrue(at_right)
                self.assertAlmostEqual(target, 1000 - 0.15 * math.degrees(1) * 0.005, places=4)
                for rate in (-150.0, 0.0):
                    self.assertFalse(self.step(ecd=ecd, rate=rate)[1])

    def test_inside_target_clips_at_right_and_left_guard(self):
        target, enabled, _, _ = self.step(ecd=7304, rate=-300, dt=0.05)
        self.assertTrue(enabled)
        self.assertAlmostEqual(target, 1000 + 360 / 8192, places=4)
        target, enabled, _, _ = self.step(ecd=7082, rate=300, dt=0.05)
        self.assertTrue(enabled)
        self.assertGreaterEqual(target, 1000 - (10 - 227 * 360 / 8192) - 0.0001)
        self.assertFalse(self.step(ecd=7081, rate=150)[1])
        target, enabled, _, _ = self.step(ecd=7081, rate=-150)
        self.assertTrue(enabled)
        self.assertAlmostEqual(target, 1000 + 0.15 * math.degrees(1) * 0.005, places=4)

    def test_actual_output_blocks_residual_torque_into_either_edge(self):
        clamp = self.library.clamp_output
        self.assertEqual(clamp(6000, 0, 10, self.guard), 0)
        self.assertEqual(clamp(-6000, 0, 10, self.guard), -6000)
        self.assertEqual(clamp(-6000, -10, 10, self.guard), 0)
        self.assertEqual(clamp(6000, -10, 10, self.guard), 6000)
        self.assertEqual(clamp(6000, -5, 10, self.guard), 6000)

    def test_inward_guard_covers_observed_six_count_release(self):
        guard = 16 * 360 / 8192
        for rate in (-150.0, 0.0):
            target, enabled, at_right, _ = self.step(ecd=7303, rate=rate, guard=guard)
            self.assertFalse(enabled)
            self.assertTrue(at_right)
            self.assertEqual(target, 1000.0)
        self.assertTrue(self.step(ecd=7303, rate=150.0, guard=guard)[1])
        relative = self.library.relative(7303, 7309)
        self.assertEqual(self.library.clamp_output(6000, relative, 10, guard), 0)

    def test_calibrated_left_edge_blocks_left_and_zero_after_restart(self):
        guard = 16 * 360 / 8192
        safe_left_travel = (7309 - 4569 - 16) * 360 / 8192
        for ecd in (4569, 4575, 4585, 4568):
            for total in (-720.0, 0.0, 1000.0):
                for rate in (150.0, 0.0):
                    with self.subTest(ecd=ecd, total=total, rate=rate):
                        target, enabled, at_right, at_left = self.step(
                            ecd=ecd, total=total, previous=total,
                            rate=rate, left=safe_left_travel, guard=guard,
                        )
                        self.assertFalse(enabled)
                        self.assertFalse(at_right)
                        self.assertTrue(at_left)
                        self.assertEqual(target, total)
            relative = self.library.relative(ecd, 7309)
            self.assertEqual(self.library.clamp_output(-6000, relative, safe_left_travel, guard), 0)
            self.assertEqual(self.library.clamp_output(6000, relative, safe_left_travel, guard), 6000)

    def test_calibrated_left_escape_right_is_gradual(self):
        guard = 16 * 360 / 8192
        safe_left_travel = (7309 - 4569 - 16) * 360 / 8192
        for ecd in (4569, 4575, 4584):
            for active in (0, 1):
                with self.subTest(ecd=ecd, active=active):
                    target, enabled, at_right, at_left = self.step(
                        ecd=ecd, previous=-2000, rate=-150,
                        left=safe_left_travel, guard=guard, active=active,
                    )
                    self.assertTrue(enabled)
                    self.assertFalse(at_right)
                    self.assertTrue(at_left)
                    self.assertAlmostEqual(target, 1000 + 0.15 * math.degrees(1) * 0.005, places=4)

    def test_calibrated_inside_target_stops_at_inward_left_guard(self):
        guard = 16 * 360 / 8192
        safe_left_travel = (7309 - 4569 - 16) * 360 / 8192
        target, enabled, _, at_left = self.step(
            ecd=4586, rate=300, left=safe_left_travel, guard=guard, dt=0.05,
        )
        self.assertTrue(enabled)
        self.assertFalse(at_left)
        self.assertAlmostEqual(target, 1000 - 360 / 8192, places=4)
        self.assertEqual(self.library.clamp_output(-6000, self.library.relative(4586, 7309),
                                                 safe_left_travel, guard), -6000)

    def test_both_calibrated_edges_wrap_across_encoder_zero(self):
        guard = 16 * 360 / 8192
        safe_left_travel = (2740 - 16) * 360 / 8192
        # Rotate the same 2740-count mechanism so its right edge is ECD 1000.
        wrapped_left = (1000 - 2740) % 8192
        target, enabled, at_right, at_left = self.step(
            ecd=wrapped_left, right=1000, rate=150,
            left=safe_left_travel, guard=guard,
        )
        self.assertFalse(enabled)
        self.assertFalse(at_right)
        self.assertTrue(at_left)
        self.assertEqual(target, 1000)
        self.assertTrue(self.step(ecd=wrapped_left, right=1000, rate=-150,
                                  left=safe_left_travel, guard=guard)[1])

    def test_invalid_inputs_fail_closed(self):
        cases = (
            {"ecd": 8192}, {"right": 8192}, {"sign": 0}, {"sign": 2},
            {"dt": -0.001}, {"dt": 0.051}, {"left": 180}, {"guard": -1},
            {"left": 0}, {"rate": float("nan")}, {"total": float("inf")},
            {"previous": float("nan")}, {"dt": float("nan")},
        )
        for case in cases:
            with self.subTest(case=case):
                self.assertFalse(self.step(**case)[1])
        for output, relative in ((float("nan"), 0), (100, float("nan")), (float("inf"), -1)):
            self.assertEqual(self.library.clamp_output(output, relative, 10, self.guard), 0)


class SourceChecks(unittest.TestCase):
    def test_python_sources_compile(self):
        for filename in ("wasd_web_teleop.py", "sentry_nav2_bridge.py"):
            path = BRINGUP / "scripts" / filename
            compile(path.read_text(encoding="utf-8"), str(path), "exec")

    def test_browser_javascript_syntax(self):
        node = shutil.which("node")
        if node is None:
            self.skipTest("node is unavailable; JavaScript syntax check skipped")
        source = (BRINGUP / "web/wasd.html").read_text(encoding="utf-8")
        script = re.search(r"<script>(.*?)</script>", source, flags=re.DOTALL).group(1)
        result = subprocess.run(
            [node, "--check", "--input-type=commonjs"],
            input=script, text=True, capture_output=True, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


class FlashVerificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from elftools.elf.elffile import ELFFile

        cls.elf_path = ROOT / "build/nyush_rm_control_h723.elf"
        hex_path = cls.elf_path.with_suffix(".hex")
        if not cls.elf_path.is_file() or not hex_path.is_file():
            raise unittest.SkipTest("Build the sentry_swerve firmware before ELF verification tests")
        cls.elf = ELFFile(io.BytesIO(cls.elf_path.read_bytes()))
        cls.symbols = {
            symbol.name: int(symbol["st_value"])
            for symbol in cls.elf.get_section_by_name(".symtab").iter_symbols()
        }
        # Use objcopy's existing Intel HEX output as an independent flash map;
        # its addresses include .data's flash initializer, not its RAM VMA.
        cls.flash_bytes = {}
        base = 0
        for line in hex_path.read_text(encoding="ascii").splitlines():
            if not line:
                continue
            record = bytes.fromhex(line.removeprefix(":"))
            if sum(record) & 0xFF or len(record) != record[0] + 5:
                raise AssertionError("Invalid build Intel HEX record")
            address = int.from_bytes(record[1:3], "big")
            kind = record[3]
            data = record[4:-1]
            if kind == 0:
                cls.flash_bytes.update((base + address + i, byte) for i, byte in enumerate(data))
            elif kind == 2:
                base = int.from_bytes(data, "big") << 4
            elif kind == 4:
                base = int.from_bytes(data, "big") << 16
            elif kind == 1:
                break

    def inspect_flash(self, mutation=None):
        flash_bytes = self.flash_bytes.copy()
        if mutation is not None:
            flash_bytes[mutation] ^= 1
        reads = []

        def read(address, length):
            reads.append((address, length))
            if 0x08000000 <= address < 0x08200000:
                return bytes(flash_bytes.get(address + i, 0xFF) for i in range(length))
            self.assertEqual((address, length), (self.symbols["aim_yaw_motor"], 4))
            return bytes(4)  # No motor has been initialized in this fake target.

        probe = SimpleNamespace(memory_read8=read, close=Mock())
        output = io.StringIO()
        arguments = SimpleNamespace(elf=str(self.elf_path), seconds=0.1)
        with patch.object(diagnostic, "_open_probe", return_value=probe), redirect_stdout(output):
            result = diagnostic.probe_status(arguments)
        probe.close.assert_called_once_with()
        return result, json.loads(output.getvalue()), reads

    def test_erased_intersection_padding_does_not_mismatch(self):
        vector = self.elf.get_section_by_name(".isr_vector")
        gap_start = int(vector["sh_addr"]) + int(vector["sh_size"])
        text_start = int(self.elf.get_section_by_name(".text")["sh_addr"])
        self.assertLess(gap_start, text_start)
        self.assertTrue(all(address not in self.flash_bytes for address in range(gap_start, text_start)))
        result, report, reads = self.inspect_flash()
        self.assertEqual(result, 4)
        self.assertTrue(report["elf_matches_flash"])
        self.assertFalse(report["registered_small_yaw"])
        self.assertEqual(reads[-1], (self.symbols["aim_yaw_motor"], 4))

    def test_text_mutation_rejects_symbols_before_ram_read(self):
        address = int(self.elf.get_section_by_name(".text")["sh_addr"]) + 32
        result, report, reads = self.inspect_flash(mutation=address)
        self.assertEqual(result, 3)
        self.assertFalse(report["elf_matches_flash"])
        self.assertEqual(report["section"], ".text")
        self.assertEqual(report["first_difference"], hex(address))
        self.assertTrue(all(0x08000000 <= start < 0x08200000 for start, _ in reads))

    def test_data_initializer_mutation_uses_flash_load_address(self):
        address = self.symbols["_sidata"]
        self.assertNotEqual(address, int(self.elf.get_section_by_name(".data")["sh_addr"]))
        result, report, reads = self.inspect_flash(mutation=address)
        self.assertEqual(result, 3)
        self.assertFalse(report["elf_matches_flash"])
        self.assertEqual(report["section"], ".data")
        self.assertEqual(report["first_difference"], hex(address))
        self.assertTrue(all(0x08000000 <= start < 0x08200000 for start, _ in reads))


if __name__ == "__main__":
    unittest.main(verbosity=2)
