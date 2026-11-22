"""Launch the loopback Foxglove Bridge and its fixed command handler together."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    directory = Path(__file__).resolve().parent
    python = os.environ['SENTRY_OPERATOR_PYTHON']
    operator = ExecuteProcess(
        cmd=[python, '-u', str(directory / 'sentry_operator.py')],
        name='sentry_operator', output='screen', sigterm_timeout='12', sigkill_timeout='6',
    )
    bridge = Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
        parameters=[str(directory / 'bridge.yaml')], output='screen',
    )
    gateway = ExecuteProcess(
        cmd=[python, '-u', str(directory / 'ws_filter.py')],
        name='sentry_ws_filter', output='screen',
    )
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(
            target_action=operator, on_exit=[Shutdown(reason='Sentry operator exited')],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge, on_exit=[Shutdown(reason='Foxglove Bridge exited')],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gateway, on_exit=[Shutdown(reason='Foxglove WebSocket filter exited')],
        )),
        operator,
        bridge,
        gateway,
    ])
