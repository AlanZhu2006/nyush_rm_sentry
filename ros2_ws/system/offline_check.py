#!/usr/bin/env python3
"""Exercise the ROS/USB boundary using a PTY, never a physical serial device."""

import binascii
import importlib.util
import math
import os
from pathlib import Path
import struct
import sys
from tempfile import TemporaryDirectory
import time

ROOT = Path(__file__).resolve().parents[2]
# The existing Python 3.10 ROS installation and the project's serial tooling
# are sufficient even while the remaining host packages await installation.
sys.path.append(str(ROOT / '.venv/lib/python3.10/site-packages'))

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


def load_module(name):
    path = ROOT / 'ros2_ws/src/mid360_nav_bringup/scripts' / f'{name}.py'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check(condition, message):
    if not condition:
        raise RuntimeError(message)
    print(f'PASS: {message}', flush=True)


def main():
    check(os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'DDS restricted to localhost')
    check(os.environ.get('ROS_DOMAIN_ID') == '181', 'isolated test domain 181')
    master_fd, slave_fd = os.openpty()
    port = os.ttyname(slave_fd)
    os.set_blocking(master_fd, False)
    vision_directory = TemporaryDirectory(prefix='sentry-offline-')
    vision_port = Path(vision_directory.name) / 'vision'
    rclpy.init(args=[
        '--ros-args', '-p', f'port:={port}', '-p', f'vision_pty:={vision_port}',
        '-p', 'telemetry_reconnect_timeout_s:=5.0',
    ])
    vision_fd = None
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        bridge_module = load_module('sentry_nav2_bridge')
        bridge = bridge_module.SentryNav2Bridge()
        nodes.append(bridge)
        adapter = load_module('fastlio_odom_adapter').FastlioOdomAdapter()
        nodes.append(adapter)
        client = Node('offline_verifier')
        nodes.append(client)
        commands = client.create_publisher(Twist, '/cmd_vel', 10)
        stops = client.create_publisher(Bool, '/emergency_stop', 10)
        poses = client.create_publisher(Odometry, '/Odometry', 10)
        wheel_messages, odom_messages, transforms = [], [], []
        client.create_subscription(Odometry, '/wheel/odom', wheel_messages.append, 10)
        client.create_subscription(Odometry, '/odom', odom_messages.append, 10)
        client.create_subscription(TFMessage, '/tf', transforms.append, 10)
        teleop_module = load_module('wasd_web_teleop')
        teleop = teleop_module.BrowserTeleop(
            topic='/cmd_vel_teleop', linear_speed=0.8, angular_speed=2.4,
            watchdog_s=0.35, publish_rate_hz=30.0)
        nodes.append(teleop)
        manual_overrides = []
        client.create_subscription(
            Bool, '/navigation_manual_override', manual_overrides.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        for node in nodes:
            executor.add_node(node)

        rx = bytearray()
        frames = []
        vision_commands = []

        def spin(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                try:
                    rx.extend(os.read(master_fd, 4096))
                except BlockingIOError:
                    pass
                while len(rx) >= 2:
                    if rx.startswith(b'SP'):
                        if len(rx) < 29:
                            break
                        vision_commands.append(bytes(rx[:29]))
                        del rx[:29]
                        continue
                    if len(rx) < 15:
                        break
                    frame = bytes(rx[:15])
                    del rx[:15]
                    if frame[:3] != b'\xa5\x5a\x01' or frame[3] not in (0x01, 0x02):
                        raise RuntimeError('Unexpected chassis or manual gimbal wire header')
                    if binascii.crc_hqx(frame[:-2], 0xffff) != struct.unpack('<H', frame[-2:])[0]:
                        raise RuntimeError('Invalid command wire CRC')
                    frames.append(struct.unpack('<2sBBHhhhBH', frame))

        def commands_of_type(packet_type):
            return [frame[4:8] for frame in frames if frame[2] == packet_type]

        def nav_frame(body):
            return body + struct.pack('<H', binascii.crc_hqx(body, 0xffff))

        def aim_telemetry(sequence, time_ms, flags):
            return nav_frame(struct.pack(
                '<2sBBHIHfffhB', b'\xa5\x5a', 1, 0x82,
                sequence, time_ms, 1234, 0.0, 0.0, 0.0, 0, flags,
            ))

        spin(0.15)
        check(bool(frames) and all(frame[4:8] == (0, 0, 0, 0) for frame in frames),
              'startup command frames are disabled zero-speed frames')
        from sentry_vision_transport import RELEASE, seal
        check(vision_commands == [RELEASE] and vision_port.is_symlink(),
              'startup releases vision and creates a private vision PTY')
        vision_fd = os.open(vision_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        vision_status = seal(b'SC' + bytes([1, 8, 33, 0]) + struct.pack('<HH', 0, 0))
        vision_telemetry = seal(struct.pack(
            '<2sB9fH', b'SP', 1, 1., 0., 0., 0., 0., 0., 0., 0., 0., 0,
        ))
        os.write(master_fd, vision_status + vision_telemetry)
        spin(0.08)
        check(os.read(vision_fd, 4096) == vision_status + vision_telemetry,
              'controller vision status and telemetry cross the shared PTY')

        teleop.control_state.claim('offline-check-client')
        os.write(master_fd, aim_telemetry(1, 900, 0x01))
        spin(0.06)
        frames.clear()
        accepted, _, _ = teleop.control_state.update(
            'offline-check-client', 0, False, ['j'], 1.0, small_yaw_armed=True)
        spin(0.15)
        check(accepted and (600, 0, 0, 1) in commands_of_type(0x02),
              'fresh controller limit feedback allows the browser small-yaw command')
        check(all(command == (0, 0, 0, 0) for command in commands_of_type(0x01)),
              'gimbal-only command leaves chassis disabled')
        os.write(master_fd, aim_telemetry(2, 925, 0x03))
        frames.clear()
        spin(0.08)
        check(commands_of_type(0x02)[-1][:3] == (0, 0, 0),
              'left limit suppresses a leftward gimbal command')
        teleop.control_state.update(
            'offline-check-client', 1, True, ['w'], 1.0,
        )
        spin(0.13)
        check(manual_overrides[-1].data,
              'browser chassis motion publishes manual takeover')
        spin(0.48)
        check(not teleop.control_state.snapshot()['armed']
              and not teleop.control_state.snapshot()['small_yaw_armed']
              and not manual_overrides[-1].data,
              'browser watchdog releases manual control')

        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = 0.2, -0.1, 0.4
        frames.clear()
        for _ in range(12):
            commands.publish(command)
            spin(0.04)
            if (200, -100, 400, 1) in commands_of_type(0x01):
                break
        check((200, -100, 400, 1) in commands_of_type(0x01),
              'ROS Twist crosses PTY as a 15-byte SI-scaled command with valid CRC')
        stops.publish(Bool(data=True))
        spin(0.08)
        frames.clear()
        commands.publish(command)
        spin(0.08)
        check(commands_of_type(0x01)
              and all(frame == (0, 0, 0, 0) for frame in commands_of_type(0x01)),
              'chassis stop disables the serial command despite active Twist input')
        stops.publish(Bool(data=False))
        spin(0.08)
        check(commands_of_type(0x01)[-1] == (0, 0, 0, 0),
              'clearing chassis stop does not replay a buffered Twist')
        commands.publish(command)
        spin(0.08)
        check(commands_of_type(0x01)[-1] == (200, -100, 400, 1),
              'fresh Twist resumes after clearing chassis stop')
        frames.clear()
        spin(0.40)
        check(commands_of_type(0x01) and commands_of_type(0x01)[-1] == (0, 0, 0, 0),
              'stale ROS input disables the controller command')

        def telemetry(sequence, time_ms, corrupt=False):
            # Encoder order is LF, RF, RR, LR in the wire protocol.
            body = struct.pack('<2sBBHI4H4hB', b'\xa5\x5a', 1, 0x81,
                               sequence, time_ms, 6141, 6814, 4081, 1358,
                               0, 0, 0, 0, 0x01)
            crc = binascii.crc_hqx(body, 0xffff) ^ int(corrupt)
            return body + struct.pack('<H', crc)

        os.write(master_fd, telemetry(1, 1000))
        spin(0.06)
        os.write(master_fd, telemetry(2, 1050))
        spin(0.06)
        check(len(wheel_messages) == 2, '29-byte telemetry frames publish /wheel/odom')
        check(wheel_messages[-1].twist.twist.linear.x == 0.0 and
              wheel_messages[-1].header.frame_id == 'odom',
              'stationary telemetry uses the expected ROS frame and velocity')
        os.write(master_fd, telemetry(3, 1100, corrupt=True))
        spin(0.06)
        check(len(wheel_messages) == 2, 'corrupted telemetry CRC is rejected')
        valid = telemetry(4, 1150)
        os.write(master_fd, valid[:7])
        spin(0.03)
        os.write(master_fd, valid[7:])
        spin(0.06)
        check(len(wheel_messages) == 3, 'parser recovers and accepts a fragmented frame')

        pose = Odometry()
        pose.header.frame_id, pose.child_frame_id = 'camera_init', 'body'
        pose.header.stamp = client.get_clock().now().to_msg()
        pose.pose.pose.position.x = 1.0
        pose.pose.pose.position.y = 2.0
        pose.pose.pose.position.z = 3.0
        # A level chassis has the measured LiDAR mount rotation in the
        # gravity-aligned FAST-LIO output, since the LiDAR and IMU axes coincide.
        half_roll, half_pitch = -0.01798 / 2, 1.03795 / 2
        orientation = pose.pose.pose.orientation
        orientation.x = math.sin(half_roll) * math.cos(half_pitch)
        orientation.y = math.cos(half_roll) * math.sin(half_pitch)
        orientation.z = -math.sin(half_roll) * math.sin(half_pitch)
        orientation.w = math.cos(half_roll) * math.cos(half_pitch)
        poses.publish(pose)
        spin(0.10)
        check(bool(odom_messages), 'FAST-LIO input produces /odom')
        result = odom_messages[-1]
        position = result.pose.pose.position
        orientation = result.pose.pose.orientation
        expected_position = (1.03277008858, 2.08750699936, 2.68209649072)
        check(all(math.isclose(actual, expected, abs_tol=1e-8) for actual, expected in
                  zip((position.x, position.y, position.z), expected_position)),
              'tilted IMU/LiDAR/base mounting offsets are applied to odometry')
        check(all(math.isclose(actual, expected, abs_tol=1e-8) for actual, expected in
                  zip((orientation.x, orientation.y, orientation.z, orientation.w), (0, 0, 0, 1))),
              'a stationary tilted IMU produces a level chassis orientation')
        check(result.header.frame_id == 'odom' and result.child_frame_id == 'base_link' and
              any(t.header.frame_id == 'odom' and t.child_frame_id == 'base_link'
                  for message in transforms for t in message.transforms),
              'odometry adapter publishes odom -> base_link TF')

        executor.remove_node(bridge)
        nodes.remove(bridge)
        frames.clear()
        vision_commands.clear()
        bridge.destroy_node()
        spin(0.03)
        check(commands_of_type(0x01) == [(0, 0, 0, 0)]
              and commands_of_type(0x02) == [(0, 0, 0, 0)]
              and vision_commands == [RELEASE] and not vision_port.exists(),
              'bridge shutdown releases vision, disables chassis and manual gimbal, and removes its PTY')
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        if vision_fd is not None:
            os.close(vision_fd)
        vision_directory.cleanup()
        os.close(slave_fd)
        os.close(master_fd)


if __name__ == '__main__':
    main()
