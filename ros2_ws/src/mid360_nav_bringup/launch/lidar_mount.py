"""Shared default MID-360 mounting geometry for mapping and navigation."""

import os


def mount_defaults():
    # On 2026-09-20, the static MID-360 IMU measured specific force
    # (-0.8583, -0.0091, 0.5061) g in the chassis's operating pose.
    # That gives URDF roll -1.03° and pitch +59.47° (+X points down).
    profiles = {
        'level': ('0.0', '0.0'),
        'tilted': ('-0.01798', '1.03795'),
    }
    profile = os.environ.get('MID360_MOUNT_PROFILE', 'tilted')
    try:
        roll, pitch = profiles[profile]
    except KeyError as error:
        raise ValueError(
            f'Unknown MID360_MOUNT_PROFILE {profile!r}. Use level or tilted.'
        ) from error
    return {
        'lidar_x': '0.0',
        'lidar_y': '-0.11',
        'lidar_z': '0.35',
        'lidar_roll': roll,
        'lidar_pitch': pitch,
        'lidar_yaw': '0.0',
    }
