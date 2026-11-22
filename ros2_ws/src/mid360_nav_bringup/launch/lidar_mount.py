"""Fixed MID-360 pose in base_link.

Translation is meters. RPY is URDF radians (Rz*Ry*Rx), matching FAST-LIO's
gravity-align formula: roll = atan2(ay, az), pitch = atan2(-ax, hypot(ay, az)).

Measured 2026-09-20 from Livox IMU specific force while the chassis was in
its operational pose: mean acc (-0.8583, -0.0091, 0.5061) g
-> roll -1.030 deg, pitch +59.470 deg. URDF +pitch points lidar +X down.
"""

LIDAR_X = '0.0'
LIDAR_Y = '-0.11'
LIDAR_Z = '0.35'
LIDAR_ROLL = '-0.01798'
LIDAR_PITCH = '1.03795'
LIDAR_YAW = '0.0'
