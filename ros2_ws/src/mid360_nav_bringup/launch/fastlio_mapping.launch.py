import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetLaunchConfiguration, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_LAUNCH_DIR = str(Path(__file__).resolve().parent)
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from lidar_mount import mount_defaults
from mid360_launch import body_to_lidar_arguments, odometry_adapter, primary_lidar_topics


def _configure_mapping_inputs(context, dual_rviz, primary_rviz):
    dual_lidar = bool(LaunchConfiguration('dual_lidar_mount_file').perform(context))
    lidar_topic, imu_topic = primary_lidar_topics(dual_lidar)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    return [
        SetLaunchConfiguration('mid360_dual_lidar', str(dual_lidar).lower()),
        SetLaunchConfiguration('mid360_primary_lidar_topic', lidar_topic),
        SetLaunchConfiguration('mid360_primary_imu_topic', imu_topic),
        SetLaunchConfiguration('mid360_primary_map_enabled', str(not dual_lidar).lower()),
        SetLaunchConfiguration('mid360_fastlio_tf', '/fastlio/tf_internal' if dual_lidar else '/tf'),
        SetLaunchConfiguration(
            'mid360_mapping_rviz', rviz_config or str(dual_rviz if dual_lidar else primary_rviz)),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_nav_bringup'))
    fast_lio_share = Path(get_package_share_directory('fast_lio'))

    robot_args = {
        name: LaunchConfiguration(name) for name in (
            'use_sim_time', 'chassis_port', 'livox_config', 'dual_lidar_mount_file',
            'wheel_odom_topic',
            'base_length', 'base_width', 'base_height', 'lidar_x', 'lidar_y',
            'lidar_z', 'lidar_roll', 'lidar_pitch', 'lidar_yaw')
    }
    robot_args.update({
        'use_ekf': 'false',
        'xfer_format': '1',
        'use_scan': LaunchConfiguration('mid360_dual_lidar'),
        'dual_lidar_fixed_frame': 'camera_init',
    })

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('chassis_port', default_value=''),
        DeclareLaunchArgument(
            'livox_config',
            default_value=str(share / 'config' / 'mid360_config.json')),
        DeclareLaunchArgument(
            'dual_lidar_mount_file',
            default_value=str(share / 'config' / 'secondary_mount_candidate.yaml'),
            description='Secondary MID-360 mount YAML. Empty uses single LiDAR.'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odom'),
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=str(fast_lio_share / 'config' / 'mid360.yaml')),
        DeclareLaunchArgument(
            'pcd_path', default_value=str(share / 'maps' / 'fastlio_map.pcd')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional RViz config override for the selected mapping mode.'),
        DeclareLaunchArgument('base_length', default_value='0.55'),
        DeclareLaunchArgument('base_width', default_value='0.55'),
        DeclareLaunchArgument('base_height', default_value='0.50'),
        *(DeclareLaunchArgument(name, default_value=value)
          for name, value in mount_defaults().items()),
        *body_to_lidar_arguments(),

        OpaqueFunction(function=_configure_mapping_inputs, args=[
            share / 'rviz' / 'dual_mapping.rviz',
            fast_lio_share / 'rviz' / 'fastlio.rviz',
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / 'launch' / 'robot.launch.py')),
            launch_arguments=robot_args.items(),
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            output='screen',
            parameters=[
                LaunchConfiguration('fastlio_config'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'map_file_path': LaunchConfiguration('pcd_path'),
                    'common.lid_topic': LaunchConfiguration('mid360_primary_lidar_topic'),
                    'common.imu_topic': LaunchConfiguration('mid360_primary_imu_topic'),
                    'preprocess.lidar_type': 1,
                    'preprocess.timestamp_unit': 3,
                    'mapping.extrinsic_est_en': False,
                    'publish.map_en': ParameterValue(
                        LaunchConfiguration('mid360_primary_map_enabled'), value_type=bool),
                    'pcd_save.pcd_save_en': ParameterValue(
                        LaunchConfiguration('mid360_primary_map_enabled'), value_type=bool),
                    'pcd_save.interval': -1,
                },
            ],
            remappings=[('/tf', LaunchConfiguration('mid360_fastlio_tf'))],
            on_exit=Shutdown(reason='FAST-LIO exited during mapping'),
        ),
        odometry_adapter('camera_init', IfCondition(LaunchConfiguration('mid360_dual_lidar'))),
        Node(
            package='mid360_nav_bringup',
            executable='dual_mid360_map',
            name='dual_mid360_map',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'map_file_path': ParameterValue(LaunchConfiguration('pcd_path'), value_type=str),
                'fixed_frame': 'camera_init',
            }],
            condition=IfCondition(LaunchConfiguration('mid360_dual_lidar')),
            on_exit=Shutdown(reason='dual MID-360 map accumulator exited'),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='fastlio_rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('mid360_mapping_rviz')],
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
