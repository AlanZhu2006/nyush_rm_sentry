#!/usr/bin/env python3
"""Exercise the ROS/USB boundary using a PTY, never a physical serial device."""

import binascii
import importlib.util
import math
import os
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
# The existing Python 3.10 ROS installation and the project's serial tooling
# are sufficient even while the remaining host packages await installation.
sys.path.append(str(ROOT / '.venv/lib/python3.10/site-packages'))

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
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
    rclpy.init(args=['--ros-args', '-p', f'port:={port}'])
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
        gimbal_preview = []
        client.create_subscription(Vector3, '/gimbal/cmd_vel_preview', gimbal_preview.append, 10)
        for node in nodes:
            executor.add_node(node)

        rx = bytearray()
        frames = []

        def spin(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                try:
                    rx.extend(os.read(master_fd, 4096))
                except BlockingIOError:
                    pass
                while len(rx) >= 15:
                    frame = bytes(rx[:15])
                    del rx[:15]
                    if frame[:4] != b'\xa5\x5a\x01\x01':
                        raise RuntimeError('Unexpected command wire header')
                    if binascii.crc_hqx(frame[:-2], 0xffff) != struct.unpack('<H', frame[-2:])[0]:
                        raise RuntimeError('Invalid command wire CRC')
                    frames.append(struct.unpack('<2sBBHhhhBH', frame))

        spin(0.15)
        check(bool(frames) and all(frame[4:8] == (0, 0, 0, 0) for frame in frames),
              'startup command frames are disabled zero-speed frames')
        teleop.control_state.claim('offline-check-client')
        accepted, _, _ = teleop.control_state.update(
            'offline-check-client', 0, False, ['i', 'j', 'u'], 1.0, True, 'manual')
        spin(0.15)
        check(accepted and bool(gimbal_preview) and
              math.isclose(gimbal_preview[-1].x, 0.25) and
              math.isclose(gimbal_preview[-1].y, 0.18) and
              math.isclose(gimbal_preview[-1].z, 0.15),
              'manual preview publishes simultaneous small yaw, pitch, and large yaw rates')
        check(all(frame[4:8] == (0, 0, 0, 0) for frame in frames),
              'gimbal-only preview leaves the chassis USB command disabled')
        teleop.control_state.update(
            'offline-check-client', 1, False, [], 1.0, True, 'cruise')
        spin(0.10)
        check(math.isclose(gimbal_preview[-1].x, 0.0) and
              math.isclose(gimbal_preview[-1].y, 0.0) and
              math.isclose(gimbal_preview[-1].z, 0.15),
              'cruise preview commands only the large yaw axis')
        teleop.control_state.update(
            'offline-check-client', 2, False, [], 1.0, True, 'scan')
        spin(0.10)
        check(gimbal_preview[-1].y > 0.0 and
              math.isclose(gimbal_preview[-1].z, 0.15),
              'combined scan preview requests pitch and large yaw together')
        spin(0.40)
        check(not teleop.control_state.snapshot()['gimbal_armed'] and
              math.isclose(gimbal_preview[-1].x, 0.0) and
              math.isclose(gimbal_preview[-1].y, 0.0) and
              math.isclose(gimbal_preview[-1].z, 0.0),
              'browser watchdog disarms gimbal preview and emits zero')
        if os.environ.get('GIMBAL_PREVIEW_ONLY') == '1':
            return
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = 0.2, -0.1, 0.4
        frames.clear()
        for _ in range(12):
            commands.publish(command)
            spin(0.04)
            if any(frame[4:8] == (200, -100, 400, 1) for frame in frames):
                break
        check(any(frame[4:8] == (200, -100, 400, 1) for frame in frames),
              'ROS Twist crosses PTY as a 15-byte SI-scaled command with valid CRC')
        frames.clear()
        spin(0.40)
        check(bool(frames) and frames[-1][4:8] == (0, 0, 0, 0),
              'stale ROS input disables the controller command')

        def telemetry(sequence, time_ms, corrupt=False):
            # Encoder order is LF, RF, LR, RR in the wire protocol.
            body = struct.pack('<2sBBHI4H4hB', b'\xa5\x5a', 1, 0x81,
                               sequence, time_ms, 6141, 6814, 4081, 1358,
                               0, 0, 0, 0, 0xff)
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
        pose.pose.pose.orientation.w = 1.0
        poses.publish(pose)
        spin(0.10)
        check(bool(odom_messages), 'FAST-LIO input produces /odom')
        result = odom_messages[-1]
        position = result.pose.pose.position
        check(all(math.isclose(actual, expected, abs_tol=1e-8) for actual, expected in
                  zip((position.x, position.y, position.z), (0.989, 2.08671, 2.69412))),
              'IMU/LiDAR/base mounting offsets are applied to odometry')
        check(result.header.frame_id == 'odom' and result.child_frame_id == 'base_link' and
              any(t.header.frame_id == 'odom' and t.child_frame_id == 'base_link'
                  for message in transforms for t in message.transforms),
              'odometry adapter publishes odom -> base_link TF')

        executor.remove_node(bridge)
        nodes.remove(bridge)
        frames.clear()
        bridge.destroy_node()
        spin(0.03)
        check(len(frames) == 3 and all(frame[4:8] == (0, 0, 0, 0) for frame in frames),
              'bridge shutdown sends three disabled frames')
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        os.close(slave_fd)
        os.close(master_fd)


if __name__ == '__main__':
    main()
