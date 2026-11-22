#!/usr/bin/env python3
"""Run the NUC's fixed operator workflows behind a local Foxglove Bridge."""

from __future__ import annotations

from collections import deque
import fcntl
import json
import math
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import threading
import time
from urllib.parse import urlencode
from urllib.request import Request, urlopen

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from logger_feed import LoggerFeed
from processes import ProcessManager


PREFIX = '/sentry/operator'
MAP_NAME = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]{0,63}')
IDENTIFIER = re.compile(r'[A-Za-z0-9_-]{8,100}')
WASD_PORT = 18088
LOGGER_PORT = 18765


def decode(message):
    try:
        value = json.loads(message.data) if len(message.data) <= 2048 else None
    except (ValueError, RecursionError):
        return None
    return value if isinstance(value, dict) else None


def identifier(value):
    return isinstance(value, str) and IDENTIFIER.fullmatch(value) is not None


class Operator(Node):
    def __init__(self, project, workspace):
        super().__init__('sentry_foxglove_operator')
        self.maps_dir = Path(os.environ.get('MID360_MAPS_DIR', workspace / 'src/mid360_nav_bringup/maps'))
        self.stack_lock = Path(os.environ.get('MID360_STACK_LOCK', workspace / '.mid360_mapping_navigation.lock'))
        self.manual_lock = None
        self.lock = threading.RLock()
        self.done = threading.Event()
        environment = {key: value for key, value in os.environ.items()
                       if key not in {'NAV_MAP_YAML', 'NAV_PCD_MAP'}}
        self.processes = ProcessManager(project, environment, self.add_log)
        self.rtt_frame = None
        self.rtt = LoggerFeed(LOGGER_PORT, self.add_log, self.add_telemetry)
        self.logs = deque(maxlen=250)
        self.receipts = {}
        self.seen = deque(maxlen=512)
        self.mode = 'idle'
        self.map_name = None
        self.started = 0.0
        self.phase_override = None
        self.message = 'Ready'
        self.busy = None
        self.owner = None
        self.owner_seen = 0.0
        self.sequence = -1
        self.claimed = None
        self.pending_drive = None
        self.was_token = None
        self.samples = {}
        self.localized = False
        self.goal_candidate = None
        self.goal_pending = False
        self.cancel_on_accept = False
        self.goal_handle = None
        self.goal_epoch = 0
        self.goal = 'idle'
        self.goal_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.status_pub = self.create_publisher(String, f'{PREFIX}/status', 1)
        self.log_pub = self.create_publisher(String, f'{PREFIX}/log', 20)
        self.rtt_pubs = {
            name: self.create_publisher(kind, f'{PREFIX}/rtt/{name}', 4)
            for name, kind in (('chassis_power_w', Float64), ('chassis_voltage_v', Float64),
                               ('wheel_rpm', Float64MultiArray), ('wheel_target_rpm', Float64MultiArray))
        }
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 5)
        self.create_subscription(String, f'{PREFIX}/command', self.command, 20)
        self.create_subscription(String, f'{PREFIX}/drive', self.drive, 1)
        self.create_subscription(PoseStamped, f'{PREFIX}/goal', self.stage_goal, 1)
        self.create_subscription(PoseWithCovarianceStamped, f'{PREFIX}/initialpose', self.seed_pose, 1)
        for topic, kind in (('/odom', Odometry), ('/wheel/odom', Odometry), ('/scan', LaserScan),
                            ('/lidar/points_fused', PointCloud2)):
            self.create_subscription(
                kind, topic, lambda msg, topic=topic: self.observe(topic, msg), qos_profile_sensor_data,
            )
        self.create_subscription(
            Bool, '/icp_probe/ready', self.observe_localized,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_timer(0.25, self.tick)
        self.create_timer(0.05, self.publish_telemetry)
        self.relay = threading.Thread(target=self.relay_drive, daemon=True)
        self.relay.start()

    def add_log(self, source, text):
        with self.lock:
            # The existing WASD banner includes its local HTTP token.
            if self.was_token:
                text = text.replace(self.was_token, '[redacted]')
            self.logs.append((source[:40], text[:1000]))

    def add_telemetry(self, frame):
        with self.lock:
            self.rtt_frame = frame

    def publish_telemetry(self):
        with self.lock:
            frame = self.rtt_frame
            self.rtt_frame = None
        if frame is None:
            return
        for name, value in (('chassis_power_w', frame.chassis_power),
                            ('chassis_voltage_v', frame.chassis_volt)):
            if math.isfinite(value):
                self.rtt_pubs[name].publish(Float64(data=float(value)))
        for name, values in (('wheel_rpm', frame.motor_rpm), ('wheel_target_rpm', frame.motor_target_rpm)):
            if len(values) == 4 and all(math.isfinite(value) for value in values):
                self.rtt_pubs[name].publish(Float64MultiArray(data=[float(value) for value in values]))

    def observe(self, topic, msg):
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        previous_stamp, _ = self.samples.get(topic, (0, 0.0))
        if stamp_ns > previous_stamp:
            self.samples[topic] = stamp_ns, time.monotonic()

    def observe_localized(self, msg):
        self.localized = bool(msg.data)

    def fresh(self, topic):
        _, arrived = self.samples.get(topic, (0, 0.0))
        return arrived > self.started and time.monotonic() - arrived < 0.9

    def map_exists(self, name):
        paths = [self.maps_dir / (name + suffix) for suffix in ('.pcd', '.pgm', '.yaml')]
        try:
            return all(path.is_file() and path.stat().st_size > 0 for path in paths)
        except OSError:
            return False

    def available_maps(self):
        return sorted(path.stem for path in self.maps_dir.glob('*.yaml')
                      if MAP_NAME.fullmatch(path.stem) and self.map_exists(path.stem))

    def stack_busy(self):
        try:
            with self.stack_lock.open('rb') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return False
        except FileNotFoundError:
            return False
        except OSError:
            return True

    def external_nodes(self):
        names = set(self.get_node_names())
        return bool(names & {'sentry_nav2_bridge', 'twist_mux', 'wasd_web_teleop'})

    def phase(self):
        if self.phase_override:
            return self.phase_override
        if self.mode == 'idle':
            if not self.processes.running('wasd') and (self.stack_busy() or self.external_nodes()):
                return 'external'
            return 'idle'
        if self.processes.poll('stack') is not None or not self.processes.running('stack'):
            return 'fault'
        if self.mode == 'map':
            ready = self.fresh('/odom') and self.fresh('/lidar/points_fused')
        else:
            ready = (self.localized and self.fresh('/odom') and self.fresh('/scan')
                     and self.fresh('/lidar/points_fused') and self.goal_client.server_is_ready())
        return 'ready' if ready else 'starting'

    def receipt(self, request_id, ok, message):
        self.receipts[request_id] = dict(id=request_id, ok=ok, message=message)
        if len(self.receipts) > 28:
            del self.receipts[next(iter(self.receipts))]
        self.message = message

    def command(self, msg):
        data = decode(msg)
        if not data or not identifier(data.get('id')):
            return
        request_id = data['id']
        action = data.get('action')
        if not isinstance(action, str):
            return
        with self.lock:
            if request_id in self.seen:
                return
            self.seen.append(request_id)
            if action == 'nav_cancel':
                self.receipt(request_id, True, self.cancel_goal())
                return
            if action == 'goal_send':
                try:
                    self.send_goal()
                    self.receipt(request_id, True, 'Goal sent to Nav2')
                except ValueError as error:
                    self.receipt(request_id, False, str(error))
                return
            handlers = {
                'map_start': lambda: self.start_stack('map', data.get('name')),
                'nav_start': lambda: self.start_stack('nav', data.get('name')),
                'map_save': self.save_map,
                'stop': self.stop_stack,
                'discard': self.discard_map,
                'manual_start': lambda: self.start_manual(data.get('owner')),
                'manual_stop': lambda: self.stop_manual(data.get('owner')),
                'logger_start': self.start_logger,
                'logger_stop': self.stop_logger,
            }
            if action not in handlers:
                self.receipt(request_id, False, 'Unknown operator command')
                return
            if self.busy:
                self.receipt(request_id, False, 'Another operation is in progress')
                return
            self.busy = threading.Thread(target=self.operate, args=(request_id, handlers[action]), daemon=True)
            self.busy.start()

    def operate(self, request_id, handler):
        try:
            result = handler()
            with self.lock:
                self.receipt(request_id, True, result)
        except (OSError, RuntimeError, ValueError) as error:
            with self.lock:
                self.receipt(request_id, False, str(error))
            self.add_log('operator', str(error))
        finally:
            with self.lock:
                self.phase_override = None
                self.busy = None

    def start_stack(self, mode, name):
        if not isinstance(name, str) or not MAP_NAME.fullmatch(name):
            raise ValueError('Use a map name with letters, numbers, dots, dashes or underscores')
        if self.mode != 'idle' or self.processes.running('stack'):
            raise RuntimeError('Stop the current mapping or navigation session first')
        if mode == 'nav' and not self.map_exists(name):
            raise ValueError('The saved map requires matching PCD, PGM and YAML files')
        if mode == 'map' and any((self.maps_dir / (name + ext)).exists()
                                 for ext in ('.pcd', '.pgm', '.yaml', '.lidar-mode')):
            raise ValueError('That map name already exists. Choose a new name to start mapping')
        if self.processes.running('wasd') or self.owner:
            self.stop_manual()
            self.done.wait(0.5)
        if self.stack_busy() or self.external_nodes():
            raise RuntimeError('Another terminal owns a ROS stack. Stop it before using Foxglove')
        with self.lock:
            self.phase_override = 'starting'
            self.samples.clear()
            self.localized = False
            self.started = time.monotonic()
            self.goal = 'idle'
            self.processes.start('stack', [mode, name, 'use_rviz:=false'], {'FASTLIO_MAP_AUTOSAVE': '0'})
            self.mode, self.map_name = mode, name
        return f'{mode} starting with {name}; waiting for live ROS data'

    def save_map(self):
        if self.mode != 'map' or self.phase() != 'ready':
            raise RuntimeError('Mapping needs live dual-LiDAR and odometry data before saving')
        self.phase_override = 'saving'
        if self.owner or self.processes.running('wasd'):
            self.stop_manual()
        self.processes.start('save', ['map-save', self.map_name])
        deadline = time.monotonic() + 120
        while self.processes.running('save') and not self.done.wait(0.1):
            if time.monotonic() >= deadline:
                self.processes.stop('save')
                raise RuntimeError('Map save timed out; mapping remains running')
        if self.processes.running('save') or self.processes.poll('save') != 0 or not self.map_exists(self.map_name):
            raise RuntimeError('Map save failed; see the log. Mapping remains running')
        return f'Saved {self.map_name} (PCD, PGM, YAML)'

    def stop_stack(self):
        if self.mode == 'map':
            self.save_map()
        return self.end_stack()

    def discard_map(self):
        if self.mode != 'map':
            raise RuntimeError('Only a mapping session can be stopped without saving')
        return self.end_stack()

    def end_stack(self):
        self.phase_override = 'stopping'
        if self.owner or self.processes.running('wasd'):
            self.stop_manual()
        with self.lock:
            self.cancel_goal()
        self.processes.stop('stack')
        with self.lock:
            self.mode, self.map_name = 'idle', None
            self.goal_epoch += 1
            self.goal_pending = False
            self.goal_handle = None
            self.goal = 'idle'
        return 'Stack stopped'

    def wasd_request(self, action, data, token):
        query = urlencode({'token': token})
        req = Request(f'http://127.0.0.1:{WASD_PORT}/api/{action}?{query}',
                      data=json.dumps(data).encode(), headers={'Content-Type': 'application/json'})
        with urlopen(req, timeout=0.2) as response:
            return json.load(response)

    def start_manual(self, owner):
        if not identifier(owner):
            raise ValueError('Invalid manual-control owner')
        if self.owner:
            raise RuntimeError('Manual control is already reserved')
        stack_ready = self.mode != 'idle' and self.phase() in {'starting', 'ready'}
        if not (self.mode == 'idle' or stack_ready):
            raise RuntimeError('Wait for the chassis bridge or stop the failed stack')
        if 'wasd_web_teleop' in self.get_node_names():
            raise RuntimeError('An existing terminal owns WASD')
        if stack_ready and not self.fresh('/wheel/odom'):
            raise RuntimeError('Waiting for live chassis telemetry')
        if not stack_ready:
            if self.external_nodes() or self.stack_busy():
                raise RuntimeError('Another terminal owns the chassis')
            self.manual_lock = self.stack_lock.open('ab')
            try:
                fcntl.flock(self.manual_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                self.manual_lock.close()
                self.manual_lock = None
                raise RuntimeError('Another terminal owns the chassis') from None
            with self.lock:
                self.samples.pop('/wheel/odom', None)
        self.was_token = secrets.token_urlsafe(24)
        try:
            self.processes.start('wasd', ['wasd', '--host', '127.0.0.1', '--port', str(WASD_PORT),
                                          '--token', self.was_token])
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline and self.processes.running('wasd') and not self.done.is_set():
                try:
                    if self.wasd_request('claim', {'client': owner}, self.was_token)['ok']:
                        break
                except (OSError, ValueError):
                    pass
                self.done.wait(0.15)
            else:
                raise RuntimeError('WASD did not become ready; see the log')
            with self.lock:
                self.owner = self.claimed = owner
                self.owner_seen = time.monotonic()
                self.sequence = -1
                self.cancel_goal()
        except (OSError, RuntimeError, ValueError):
            self.stop_manual()
            raise
        return 'Manual control reserved. Hold a direction to move'

    def stop_manual(self, owner=None):
        with self.lock:
            if owner is not None and self.owner != owner:
                raise RuntimeError('This panel does not own manual control')
            old_owner, token = self.claimed, self.was_token
            self.owner = self.claimed = self.pending_drive = None
        if old_owner and token:
            try:
                self.wasd_request('release', {'client': old_owner}, token)
            except (OSError, ValueError):
                pass  # The WASD watchdog also expires the lease.
        try:
            self.processes.stop('wasd')
        finally:
            if self.manual_lock:
                self.manual_lock.close()
                self.manual_lock = None
            self.was_token = None
        return 'Manual control released'

    def drive(self, msg):
        data = decode(msg)
        if not data:
            return
        seq = data.get('seq')
        axes = [data.get(key) for key in ('forward', 'left', 'turn')]
        if type(seq) is not int or not all(type(axis) in (int, float) and axis in (-1, 0, 1) for axis in axes):
            return
        now = time.monotonic()
        with self.lock:
            if not self.owner or self.owner != data.get('owner') or seq <= self.sequence:
                return
            self.sequence = seq
            self.owner_seen = now
            if not self.fresh('/wheel/odom') or self.phase_override in {'saving', 'stopping'}:
                axes = [0, 0, 0]
            keys = [pair[axis < 0] for pair, axis in zip(('ws', 'ad', 'qe'), axes) if axis]
            self.pending_drive = (now, self.was_token, {
                'client': self.owner, 'sequence': seq, 'armed': bool(keys), 'keys': keys, 'speed_scale': 0.5,
            })

    def relay_drive(self):
        while not self.done.wait(0.025):
            with self.lock:
                pending = self.pending_drive
                self.pending_drive = None
            if not pending:
                continue
            received, token, payload = pending
            if time.monotonic() - received > 0.2 or token is None:
                continue
            try:
                self.wasd_request('control', payload, token)
            except (OSError, ValueError):
                with self.lock:
                    if self.owner == payload['client'] and self.was_token == token:
                        self.owner_seen = 0.0

    def start_logger(self):
        if self.processes.running('logger'):
            raise RuntimeError('Logger is already running')
        if subprocess.run(['pgrep', '-f', r'rtt_(ws_bridge|dashboard)\.py'],
                          stdout=subprocess.DEVNULL, check=False).returncode == 0:
            raise RuntimeError('Another terminal is using the RTT logger')
        port = str(LOGGER_PORT)
        self.processes.start('logger', ['logger', '--connect', 'normal', '--ws-host', '127.0.0.1',
                                        '--ws-port', port, '--http-host', '127.0.0.1', '--http-port', port])
        self.rtt.start()
        return 'RTT logger starting; the log will report probe connection'

    def stop_logger(self):
        self.rtt.stop()
        with self.lock:
            self.rtt_frame = None
        self.processes.stop('logger')
        return 'RTT logger stopped'

    def valid_pose(self, message, pose):
        return (self.mode == 'nav' and message.header.frame_id == 'map'
                and all(math.isfinite(value) for value in
                        (pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z, pose.orientation.w))
                and abs(math.sqrt(sum(getattr(pose.orientation, axis)**2 for axis in 'xyzw')) - 1) < 0.01)

    def stage_goal(self, msg):
        if self.valid_pose(msg, msg.pose):
            self.goal_candidate = msg
            self.message = 'Goal staged. Press Send goal after checking the location'

    def seed_pose(self, msg):
        if (self.valid_pose(msg, msg.pose.pose) and self.phase() in {'starting', 'ready'}
                and all(math.isfinite(value) for value in msg.pose.covariance)):
            self.cancel_goal()
            self.localized = False
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(msg)
            self.message = 'Initial pose sent to ICP; waiting for localization'

    def send_goal(self):
        if (self.mode != 'nav' or self.phase() != 'ready' or not self.fresh('/wheel/odom')
                or self.owner or self.busy):
            raise ValueError('Goal requires live navigation and chassis telemetry with manual control released')
        if self.goal_pending or self.goal_handle:
            raise ValueError('Cancel the active goal first')
        if self.goal_candidate is None:
            raise ValueError('Choose a goal with the Foxglove 3D 2D Pose tool first')
        pose = self.goal_candidate
        self.goal_candidate = None
        pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pending, self.cancel_on_accept = True, False
        self.goal = 'sending'
        future = self.goal_client.send_goal_async(NavigateToPose.Goal(pose=pose))
        epoch = self.goal_epoch
        future.add_done_callback(lambda result: self.goal_response(result, epoch))

    def goal_response(self, future, epoch):
        with self.lock:
            if epoch != self.goal_epoch:
                return
            self.goal_pending = False
            try:
                handle = future.result()
            except Exception as error:
                self.add_log('navigation', f'Goal request failed: {error}')
                self.goal = 'rejected'
                return
            if not handle or not handle.accepted:
                self.goal = 'rejected'
                return
            self.goal_handle = handle
            self.goal = 'active'
            handle.get_result_async().add_done_callback(lambda result: self.goal_result(result, epoch))
            if self.cancel_on_accept or self.mode != 'nav':
                self.cancel_goal()

    def goal_result(self, future, epoch):
        with self.lock:
            if epoch != self.goal_epoch:
                return
            self.goal_handle = None
            try:
                status = future.result().status
            except Exception as error:
                self.add_log('navigation', f'Goal result unavailable: {error}')
                status = GoalStatus.STATUS_UNKNOWN
            self.goal = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                         GoalStatus.STATUS_CANCELED: 'canceled'}.get(status, 'aborted')

    def cancel_goal(self):
        self.goal_candidate = None
        self.cancel_on_accept = True
        if self.goal_handle:
            self.goal = 'canceling'
            self.goal_handle.cancel_goal_async()
            return 'Goal cancellation requested'
        return 'Staged goal cleared'

    def tick(self):
        with self.lock:
            if self.owner and (not self.processes.running('wasd') or time.monotonic() - self.owner_seen > 2.5):
                if not self.busy:
                    self.busy = threading.Thread(
                        target=self.operate, args=(secrets.token_hex(12), self.stop_manual), daemon=True,
                    )
                    self.busy.start()
            phase = self.phase()
            goal_target = None
            if self.goal_candidate:
                p = self.goal_candidate.pose.position
                goal_target = f'{p.x:.2f}, {p.y:.2f} m'
            status = dict(mode=self.mode, phase=phase, map=self.map_name, maps=self.available_maps(),
                          manual=self.owner is not None, manual_owner=self.owner,
                          logger=self.processes.running('logger'), logger_attached=self.rtt.connected,
                          serial=self.fresh('/wheel/odom'), busy=self.busy is not None,
                          goal=self.goal, goal_target=goal_target, message=self.message,
                          receipts=list(self.receipts.values()))
            self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
            for _ in range(min(len(self.logs), 20)):
                source, text = self.logs.popleft()
                self.log_pub.publish(String(data=json.dumps(dict(source=source, text=text), ensure_ascii=False)))

    def close(self):
        self.done.set()
        if self.busy:
            self.busy.join(timeout=3)
        self.stop_manual()
        self.stop_logger()
        self.processes.close()


def run(project, workspace):
    shutdown = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: shutdown.set())
    node = Operator(project, workspace)
    try:
        while not shutdown.is_set():
            rclpy.spin_once(node, timeout_sec=0.15)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    project = Path(__file__).resolve().parents[2]
    workspace = Path(os.environ['ROBOT_WS']).resolve()
    with (workspace / '.foxglove-operator.lock').open('ab') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('A Foxglove operator is already running in this workspace') from None
        run(project, workspace)


if __name__ == '__main__':
    main()
