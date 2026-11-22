#!/usr/bin/env python3
"""Gate autonomous velocity before the chassis command mux."""

import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool


NAVIGATION_ACTIONS = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints')
ACTIVE_GOAL_STATES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
TERMINAL_GOAL_STATES = (
    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
)


class NavigationGuard(Node):
    def __init__(self):
        super().__init__('navigation_guard')
        persistent_state = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(Twist, '/cmd_vel_nav_ready', 1)
        self._localized = False
        self._manual_override = False
        self._emergency_stop = False
        self._forwarding = False
        self._freshness = {'odometry': (0, 0.0), 'scan': (0, 0.0)}
        dual_topics = ('/lidar/points_fused', '/lidar/points_primary', '/lidar/points_secondary')
        if self.declare_parameter('require_dual_lidar', False).value:
            for topic in dual_topics:
                self._freshness[topic] = (0, 0.0)
                self.create_subscription(
                    PointCloud2, topic, lambda msg, topic=topic: self._observe(topic, msg.header.stamp),
                    qos_profile_sensor_data,
                )
        self._cancel_clients = {
            name: self.create_client(CancelGoal, f'/{name}/_action/cancel_goal')
            for name in NAVIGATION_ACTIONS
        }
        self._goal_states = {name: {} for name in NAVIGATION_ACTIONS}
        self._interrupted_goals = {name: set() for name in NAVIGATION_ACTIONS}
        self._pending_cancel = set()
        self._cancel_requests = {}
        self._canceling = {name: set() for name in NAVIGATION_ACTIONS}
        for name in NAVIGATION_ACTIONS:
            self.create_subscription(
                GoalStatusArray, f'/{name}/_action/status',
                lambda msg, name=name: self._on_status(name, msg), persistent_state,
            )
        self.create_subscription(Bool, '/icp_probe/ready', self._on_ready, persistent_state)
        self.create_subscription(
            Bool, '/navigation_manual_override', self._on_manual, persistent_state,
        )
        self.create_subscription(Bool, '/emergency_stop', self._on_stop, 10)
        self.create_subscription(
            Odometry, '/odom', lambda msg: self._observe('odometry', msg.header.stamp),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, '/scan', lambda msg: self._observe('scan', msg.header.stamp),
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, '/cmd_vel_nav_smoothed', self._on_velocity, 1)
        self.create_timer(0.05, self._check_motion, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.5, self._cancel_goals, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _observe(self, name, stamp):
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns > self._freshness[name][0]:
            self._freshness[name] = stamp_ns, time.monotonic()

    def _allowed(self):
        if (not self._localized or self._manual_override or self._emergency_stop
                or self._pending_cancel or any(self._canceling.values())):
            return False
        if not any(
            state == GoalStatus.STATUS_EXECUTING and goal not in self._interrupted_goals[name]
            for name, states in self._goal_states.items() for goal, state in states.items()
        ):
            return False
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        return all(
            received > 0.0 and now - received <= 0.5
            and -100_000_000 <= ros_now - stamp <= 500_000_000
            for stamp, received in self._freshness.values()
        )

    def _check_motion(self):
        if self._forwarding and not self._allowed():
            self._forwarding = False
            self._publisher.publish(Twist())

    def _on_velocity(self, message):
        if self._allowed():
            self._publisher.publish(message)
            self._forwarding = True
        else:
            self._check_motion()

    def _on_ready(self, message):
        lost_alignment = self._localized and not message.data
        self._localized = message.data
        if lost_alignment:
            self._request_cancel('localization reset')
        self._check_motion()

    def _on_manual(self, message):
        takeover = message.data and not self._manual_override
        self._manual_override = message.data
        if takeover:
            self._request_cancel('manual chassis takeover')
        self._check_motion()

    def _on_stop(self, message):
        stopped = message.data and not self._emergency_stop
        self._emergency_stop = message.data
        if stopped:
            self._request_cancel('chassis stop')
        self._check_motion()

    def _request_cancel(self, reason):
        self.get_logger().info(f'Canceling navigation goals after {reason}')
        for name, states in self._goal_states.items():
            self._interrupted_goals[name].update(
                goal for goal, state in states.items() if state in ACTIVE_GOAL_STATES
            )
        self._pending_cancel.update(NAVIGATION_ACTIONS)
        self._cancel_goals()

    def _on_status(self, name, message):
        states = {bytes(goal.goal_info.goal_id.uuid): goal.status for goal in message.status_list}
        previous_states = self._goal_states[name]
        self._goal_states[name] = states
        finished = {goal for goal, state in states.items() if state in TERMINAL_GOAL_STATES}
        self._canceling[name].difference_update(finished)
        if self._manual_override or self._emergency_stop:
            self._interrupted_goals[name].update(
                goal for goal, state in states.items() if state in ACTIVE_GOAL_STATES
            )
            if any(state in ACTIVE_GOAL_STATES and previous_states.get(goal) not in ACTIVE_GOAL_STATES
                   for goal, state in states.items()):
                self._pending_cancel.add(name)
        self._interrupted_goals[name].difference_update(finished)

    def _cancel_goals(self):
        for name in self._pending_cancel:
            client = self._cancel_clients[name]
            if name not in self._cancel_requests and client.service_is_ready():
                future = client.call_async(CancelGoal.Request())
                self._cancel_requests[name] = future
                future.add_done_callback(lambda result, name=name: self._cancel_response(name, result))

    def _cancel_response(self, name, future):
        del self._cancel_requests[name]
        result = future.result()
        if result.return_code != CancelGoal.Response.ERROR_NONE:
            self.get_logger().warning(f'{name} rejected cancellation; autonomous output stays blocked',
                                      throttle_duration_sec=2.0)
            return
        self._pending_cancel.discard(name)
        for goal in result.goals_canceling:
            goal_id = bytes(goal.goal_id.uuid)
            if self._goal_states[name].get(goal_id) not in TERMINAL_GOAL_STATES:
                self._interrupted_goals[name].add(goal_id)
                self._canceling[name].add(goal_id)
        if (self._manual_override or self._emergency_stop) and any(
                state in ACTIVE_GOAL_STATES and goal_id not in self._canceling[name]
                for goal_id, state in self._goal_states[name].items()):
            self._pending_cancel.add(name)


def main():
    rclpy.init()
    node = NavigationGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
