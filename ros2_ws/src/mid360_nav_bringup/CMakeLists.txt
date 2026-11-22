cmake_minimum_required(VERSION 3.8)
project(mid360_nav_bringup)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(dual_mid360_fusion src/dual_mid360_fusion.cpp)
target_compile_features(dual_mid360_fusion PRIVATE cxx_std_17)
target_compile_options(dual_mid360_fusion PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(dual_mid360_fusion
  geometry_msgs livox_ros_driver2 rclcpp sensor_msgs tf2 tf2_ros
)
install(TARGETS dual_mid360_fusion DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config urdf maps rviz web
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_navigation.sh
  scripts/fastlio_odom_adapter.py
  scripts/navigation_guard.py
  scripts/save_map.sh
  scripts/sentry_nav2_bridge.py
  scripts/sentry_vision_transport.py
  scripts/start_mapping.sh
  scripts/start_navigation.sh
  scripts/start_wasd.sh
  scripts/wasd_teleop.py
  scripts/wasd_web_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
