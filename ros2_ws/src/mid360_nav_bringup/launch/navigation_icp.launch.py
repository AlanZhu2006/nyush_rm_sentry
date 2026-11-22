import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
import yaml

_LAUNCH_DIR = str(Path(__file__).resolve().parent)
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from lidar_mount import mount_defaults
from mid360_launch import body_to_lidar_arguments, odometry_adapter, primary_lidar_topics


def _dual_nav_params(context, params_file):
    path = Path(params_file.perform(context)).expanduser()
    try:
        config = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f'Cannot read dual LiDAR Nav2 parameters {path}: {error}') from error

    observations = {
        'lidar_primary': ('/lidar/points_primary', 'livox_frame'),
        'lidar_secondary': ('/lidar/points_secondary', 'livox_secondary_frame'),
    }
    rewrites = {}
    for costmap, layer in [('local_costmap', 'voxel_layer'),
                           ('global_costmap', 'obstacle_layer')]:
        try:
            parameters = config[costmap][costmap]['ros__parameters']
            layer_config = parameters[layer]
        except (KeyError, TypeError) as error:
            raise ValueError(f'{path}: missing Nav2 {costmap}.{layer} parameters') from error

        layer_name = f'{costmap}.{layer}'
        if not isinstance(layer_config, dict):
            raise ValueError(f'{path}: missing Nav2 {layer_name} parameters')
        plugins = parameters.get('plugins')
        if (not isinstance(plugins, list) or layer not in plugins
                or layer_config.get('enabled') is False):
            raise ValueError(f'{path}: enable {layer_name} for dual LiDAR navigation')
        if not isinstance(layer_config.get('observation_sources'), str):
            raise ValueError(f'{path}: {layer_name} needs a string observation_sources parameter')
        for source, (topic, frame) in observations.items():
            info = layer_config.get(source)
            if (not isinstance(info, dict) or info.get('topic') != topic
                    or info.get('data_type') != 'PointCloud2'
                    or info.get('sensor_frame', '') not in ('', frame)):
                raise ValueError(
                    f'{path}: {layer_name}.{source} must use {topic} (PointCloud2, frame {frame})')
        rewrites[f'{costmap}.{costmap}.ros__parameters.{layer}.observation_sources'] = (
            ' '.join(observations))

    return RewrittenYaml(source_file=str(path), param_rewrites=rewrites)


def _icp_cloud_topic(context, dual_lidar):
    mode = LaunchConfiguration('icp_cloud_mode').perform(context)
    if mode == 'auto':
        pcd_map = Path(LaunchConfiguration('pcd_map').perform(context)).expanduser()
        mode_file = pcd_map.with_suffix('.lidar-mode')
        try:
            mode = mode_file.read_text(encoding='utf-8').strip()
        except FileNotFoundError:
            mode = 'primary'
        except (OSError, UnicodeError) as error:
            raise ValueError(f'Cannot read LiDAR mode for {pcd_map}: {error}') from error
        if mode not in ('primary', 'single', 'dual'):
            raise ValueError(f'Invalid LiDAR mode {mode!r} in {mode_file}')

    if mode == 'dual':
        if not dual_lidar:
            raise ValueError('icp_cloud_mode:=dual requires a secondary LiDAR mount file')
        return '/lidar/points_fused'
    return '/cloud_registered_body'


def _configure_lidar_topics(context):
    dual_lidar = bool(LaunchConfiguration('dual_lidar_mount_file').perform(context))
    lidar_topic, imu_topic = primary_lidar_topics(dual_lidar)
    icp_cloud_topic = _icp_cloud_topic(context, dual_lidar)
    params_file = LaunchConfiguration('params_file')
    if dual_lidar:
        params_file = _dual_nav_params(context, params_file)
    scan_cloud_topic = '/lidar/points_fused' if dual_lidar else '/cloud_registered_body'

    return [
        SetLaunchConfiguration('mid360_dual_lidar', str(dual_lidar).lower()),
        SetLaunchConfiguration('mid360_primary_lidar_topic', lidar_topic),
        SetLaunchConfiguration('mid360_primary_imu_topic', imu_topic),
        SetLaunchConfiguration('mid360_scan_cloud_topic', scan_cloud_topic),
        SetLaunchConfiguration('mid360_icp_cloud_topic', icp_cloud_topic),
        SetLaunchConfiguration('mid360_nav_params_file', params_file),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_nav_bringup'))
    fast_lio_share = Path(get_package_share_directory('fast_lio'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))

    robot_args = {
        name: LaunchConfiguration(name) for name in (
            'use_sim_time', 'chassis_port', 'livox_config', 'dual_lidar_mount_file',
            'wheel_odom_topic', 'base_length', 'base_width', 'base_height',
            'lidar_x', 'lidar_y', 'lidar_z', 'lidar_roll', 'lidar_pitch',
            'lidar_yaw')
    }
    robot_args.update({
        'use_ekf': 'false',
        'xfer_format': '1',
        'use_scan': 'false',
        'dual_lidar_fixed_frame': 'odom',
        'navigation_command_topic': '/cmd_vel_nav_ready',
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
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=str(fast_lio_share / 'config' / 'mid360.yaml')),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odom'),
        DeclareLaunchArgument(
            'map', default_value=str(share / 'maps' / 'fastlio_map.yaml')),
        DeclareLaunchArgument(
            'pcd_map', default_value=str(share / 'maps' / 'fastlio_map.pcd')),
        DeclareLaunchArgument(
            'icp_cloud_mode', default_value='auto', choices=['auto', 'primary', 'dual'],
            description='Auto uses a fused ICP cloud for maps with a dual LiDAR mode marker.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share / 'config' / 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'icp_params_file',
            default_value=str(share / 'config' / 'icp_localization.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(share / 'rviz' / 'nav2_vnc.rviz')),
        DeclareLaunchArgument('base_length', default_value='0.55'),
        DeclareLaunchArgument('base_width', default_value='0.55'),
        DeclareLaunchArgument('base_height', default_value='0.50'),
        *(DeclareLaunchArgument(name, default_value=value)
          for name, value in mount_defaults().items()),
        *body_to_lidar_arguments(),
        DeclareLaunchArgument('icp_auto_start', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('fitness_threshold', default_value='0.10'),
        DeclareLaunchArgument('xy_search_steps', default_value='3'),
        DeclareLaunchArgument('yaw_offset_deg', default_value='180.0'),

        OpaqueFunction(function=_configure_lidar_topics),
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
                    'common.lid_topic': LaunchConfiguration('mid360_primary_lidar_topic'),
                    'common.imu_topic': LaunchConfiguration('mid360_primary_imu_topic'),
                    'preprocess.lidar_type': 1,
                    'preprocess.timestamp_unit': 3,
                    'mapping.extrinsic_est_en': False,
                    'publish.path_en': False,
                    'publish.map_en': False,
                    'pcd_save.pcd_save_en': False,
                },
            ],
            # The upstream FAST-LIO build publishes camera_init->body. The
            # adapter below publishes the authoritative odom->base_link TF.
            remappings=[('/tf', '/fastlio/tf_internal')],
            on_exit=Shutdown(reason='FAST-LIO exited; no odometry fallback'),
        ),
        odometry_adapter('odom'),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                str(share / 'config' / 'pointcloud_to_laserscan.yaml'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                },
            ],
            remappings=[
                ('cloud_in', LaunchConfiguration('mid360_scan_cloud_topic')),
                ('scan', '/scan'),
            ],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'yaml_filename': LaunchConfiguration('map'),
                },
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_icp_localization',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'), value_type=bool),
                'node_names': ['map_server'],
            }],
        ),
        Node(
            package='icp_probe',
            executable='direct_icp_probe',
            name='direct_icp_probe',
            output='screen',
            parameters=[
                LaunchConfiguration('icp_params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'map_path': LaunchConfiguration('pcd_map'),
                    'input_topic': LaunchConfiguration('mid360_icp_cloud_topic'),
                    'auto_start': ParameterValue(
                        LaunchConfiguration('icp_auto_start'), value_type=bool),
                    'initial_x': ParameterValue(
                        LaunchConfiguration('initial_x'), value_type=float),
                    'initial_y': ParameterValue(
                        LaunchConfiguration('initial_y'), value_type=float),
                    'initial_z': ParameterValue(
                        LaunchConfiguration('initial_z'), value_type=float),
                    'initial_yaw': ParameterValue(
                        LaunchConfiguration('initial_yaw'), value_type=float),
                    'fitness_threshold': ParameterValue(
                        LaunchConfiguration('fitness_threshold'), value_type=float),
                    'xy_search_steps': ParameterValue(
                        LaunchConfiguration('xy_search_steps'), value_type=int),
                    'yaw_offset_deg': ParameterValue(
                        LaunchConfiguration('yaw_offset_deg'), value_type=float),
                },
            ],
            on_exit=Shutdown(reason='ICP localization exited'),
        ),
        Node(
            package='mid360_nav_bringup',
            executable='navigation_guard.py',
            name='navigation_guard',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'require_dual_lidar': ParameterValue(
                    LaunchConfiguration('mid360_dual_lidar'), value_type=bool),
            }],
            on_exit=Shutdown(reason='navigation command guard exited'),
        ),
        GroupAction([
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            SetRemap(src='cmd_vel_smoothed', dst='/cmd_vel_nav_smoothed'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2_share / 'launch' / 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('mid360_nav_params_file'),
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': 'False',
                }.items(),
            ),
        ]),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            # Xtigervnc uses CPU-based llvmpipe rendering. Keep visualization
            # below the priority of FAST-LIO and Nav2 control callbacks.
            prefix=['nice -n 10'],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            on_exit=Shutdown(reason='rviz exited'),
        ),
    ])
