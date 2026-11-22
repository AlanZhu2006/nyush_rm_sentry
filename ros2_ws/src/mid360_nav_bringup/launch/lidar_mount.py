"""Shared default MID-360 mounting geometry for mapping and navigation."""

import os


def mount_defaults():
    # The tilted pose was measured on the September 20 hardware setup.
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
