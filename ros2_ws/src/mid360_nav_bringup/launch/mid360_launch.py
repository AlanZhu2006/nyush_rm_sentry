"""Shared FAST-LIO input and chassis-frame setup."""

from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def primary_lidar_topics(dual_lidar):
    if dual_lidar:
        return '/livox/lidar_192_168_1_182', '/livox/imu_192_168_1_182'
    return '/livox/lidar', '/livox/imu'


def body_to_lidar_arguments():
    # FAST-LIO mid360.yaml expresses the LiDAR pose in the IMU body frame.
    defaults = {
        'x': '-0.011', 'y': '-0.02329', 'z': '0.04412',
        'roll': '0.0', 'pitch': '0.0', 'yaw': '0.0',
    }
    return [DeclareLaunchArgument(f'body_to_lidar_{axis}', default_value=value)
            for axis, value in defaults.items()]


def odometry_adapter(odom_frame, condition=None):
    parameters = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'source_odom_topic': '/Odometry',
        'output_odom_topic': '/odom',
        'source_world_frame': 'camera_init',
        'source_body_frame': 'body',
        'odom_frame': odom_frame,
        'base_frame': 'base_link',
    }
    for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        parameters[f'base_to_lidar_{axis}'] = ParameterValue(
            LaunchConfiguration(f'lidar_{axis}'), value_type=float)
        key = f'body_to_lidar_{axis}'
        parameters[key] = ParameterValue(LaunchConfiguration(key), value_type=float)

    return Node(
        package='mid360_nav_bringup',
        executable='fastlio_odom_adapter.py',
        name='fastlio_odom_adapter',
        output='screen',
        parameters=[parameters],
        condition=condition,
        on_exit=Shutdown(reason='FAST-LIO odometry adapter exited; no wheel fallback'),
    )
