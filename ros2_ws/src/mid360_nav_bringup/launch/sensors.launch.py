from math import isfinite
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _load_secondary_mount(filename):
    path = Path(filename).expanduser()
    try:
        pose = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f'Cannot read dual LiDAR mount file {path}: {error}') from error

    if not isinstance(pose, dict) or set(pose) != {'xyz', 'rpy'}:
        raise ValueError(f'{path} must contain xyz (meters) and rpy (radians)')

    def read_vector(key):
        values = pose[key]
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError(f'{path}: {key} must contain three finite real numbers')
        try:
            valid = all(type(value) in (int, float) and isfinite(value) for value in values)
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f'{path}: {key} must contain three finite real numbers')
        return tuple(str(float(value)) for value in values)

    return read_vector('xyz'), read_vector('rpy')


def _sensor_pipeline(context, dual_livox_config, scan_config):
    use_sim_time = LaunchConfiguration('use_sim_time')
    livox_config = LaunchConfiguration('livox_config')
    xfer_format = ParameterValue(LaunchConfiguration('xfer_format'), value_type=int)
    scan_cloud_topic = LaunchConfiguration('scan_cloud_topic')
    mount_file = LaunchConfiguration('dual_lidar_mount_file').perform(context)
    actions = []

    if mount_file:
        xyz, rpy = _load_secondary_mount(mount_file)
        livox_config = str(dual_livox_config)
        xfer_format = 1
        scan_cloud_topic = '/lidar/points_fused'
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_secondary_mount',
            arguments=[
                '--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                '--roll', rpy[0], '--pitch', rpy[1], '--yaw', rpy[2],
                '--frame-id', 'base_link', '--child-frame-id', 'livox_secondary_frame',
            ],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        ))

    actions.append(Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': xfer_format,
            'multi_topic': int(bool(mount_file)),
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '',
            'user_config_path': livox_config,
            'cmdline_input_bd_code': 'livox0000000001',
            'use_sim_time': use_sim_time,
        }],
        on_exit=Shutdown(reason='dual MID-360 driver exited') if mount_file else None,
    ))

    if mount_file:
        actions.append(Node(
            package='mid360_nav_bringup',
            executable='dual_mid360_fusion',
            name='dual_mid360_fusion',
            output='screen',
            parameters=[{
                'fixed_frame': ParameterValue(
                    LaunchConfiguration('dual_lidar_fixed_frame'), value_type=str),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            on_exit=Shutdown(reason='dual MID-360 fusion exited'),
        ))

    actions.append(Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_scan')),
        parameters=[str(scan_config), {'use_sim_time': use_sim_time}],
        remappings=[('cloud_in', scan_cloud_topic), ('scan', '/scan')],
    ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_nav_bringup'))
    xacro_file = share / 'urdf' / 'robot.urdf.xacro'
    default_livox_config = share / 'config' / 'mid360_config.json'
    dual_livox_config = share / 'config' / 'dual_mid360_config.json'
    scan_config = share / 'config' / 'pointcloud_to_laserscan.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command([
            'xacro ', str(xacro_file),
            ' base_length:=', LaunchConfiguration('base_length'),
            ' base_width:=', LaunchConfiguration('base_width'),
            ' base_height:=', LaunchConfiguration('base_height'),
            ' lidar_x:=', LaunchConfiguration('lidar_x'),
            ' lidar_y:=', LaunchConfiguration('lidar_y'),
            ' lidar_z:=', LaunchConfiguration('lidar_z'),
            ' lidar_roll:=', LaunchConfiguration('lidar_roll'),
            ' lidar_pitch:=', LaunchConfiguration('lidar_pitch'),
            ' lidar_yaw:=', LaunchConfiguration('lidar_yaw'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('livox_config', default_value=str(default_livox_config)),
        DeclareLaunchArgument('xfer_format', default_value='0'),
        DeclareLaunchArgument('use_scan', default_value='true'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument(
            'dual_lidar_mount_file', default_value='',
            description='Secondary MID-360 mount YAML with xyz in meters and rpy in radians'),
        DeclareLaunchArgument(
            'dual_lidar_fixed_frame', default_value='',
            description='Fixed TF frame for aligning dual LiDAR observations in motion'),
        DeclareLaunchArgument('base_length', default_value='0.55'),
        DeclareLaunchArgument('base_width', default_value='0.55'),
        DeclareLaunchArgument('base_height', default_value='0.50'),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='-0.11'),
        DeclareLaunchArgument('lidar_z', default_value='0.35'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        OpaqueFunction(function=_sensor_pipeline, args=[dual_livox_config, scan_config]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),
    ])
