#!/usr/bin/env python3
"""Check installed Nav2/ICP/RViz with synthetic inputs, never a USB bridge.

This is a dependency and lifecycle smoke test, not a sensor/vehicle simulation.
No navigation goal is sent. All children are scoped to localhost domain 182.
"""

import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def require(condition, message):
    if not condition:
        raise RuntimeError(message)
    print(f"PASS: {message}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Skip VNC/RViz checks")
    arguments = parser.parse_args()
    require(os.environ.get("ROS_LOCALHOST_ONLY") == "1", "localhost-only DDS")
    require(os.environ.get("ROS_DOMAIN_ID") == "182", "isolated runtime domain 182")
    require(os.environ.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp",
            "production Cyclone DDS middleware")
    share = Path(get_package_share_directory("mid360_nav_bringup"))
    params = share / "config/nav2_params.yaml"
    logs = Path(tempfile.mkdtemp(prefix="nyush-sentry-runtime-check."))
    print(f"Runtime logs: {logs}", flush=True)
    child_env = dict(os.environ, ROS_LOG_DIR=str(logs / "ros"))
    children = []

    def start(label, command):
        log = (logs / f"{label}.log").open("w")
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=child_env, start_new_session=True, cwd=logs)
        children.append((label, process, log))

    def run_node(label, package, executable, *arguments):
        binary = Path(get_package_prefix(package)) / "lib" / package / executable
        start(label, [str(binary), "--ros-args", *map(str, arguments)])

    rclpy.init()
    node = rclpy.create_node("sentry_runtime_check")
    try:
        # Refuse to share the fixture domain with a pre-existing stack.
        until = time.monotonic() + 1.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
        require(node.get_node_names() == ["sentry_runtime_check"],
                "test domain has no pre-existing nodes")
        for launch_file in ("navigation_icp.launch.py", "fastlio_mapping.launch.py"):
            result = subprocess.run(
                ["ros2", "launch", "mid360_nav_bringup", launch_file, "--show-args"],
                capture_output=True, text=True, env=child_env, timeout=20)
            (logs / f"{launch_file}.args").write_text(result.stdout + result.stderr)
            require(result.returncode == 0, f"installed launch resolves: {launch_file}")

        from livox_ros_driver2.msg import CustomMsg
        type(CustomMsg).__import_type_support__()
        require(type(CustomMsg)._TYPE_SUPPORT is not None,
                "Livox custom message type support loads")
        for package, executable in (("fast_lio", "fastlio_mapping"),
                                    ("livox_ros_driver2", "livox_ros_driver2_node"),
                                    ("icp_probe", "direct_icp_probe"),
                                    ("mid360_nav_bringup", "dual_mid360_fusion"),
                                    ("mid360_nav_bringup", "dual_mid360_map")):
            binary = Path(get_package_prefix(package)) / "lib" / package / executable
            result = subprocess.run(["ldd", str(binary)], capture_output=True,
                                    text=True, env=child_env, timeout=10)
            require(result.returncode == 0 and "not found" not in result.stdout,
                    f"all shared libraries resolve: {package}")

        if not arguments.headless:
            with socket.create_connection(("127.0.0.1", 5902), timeout=3) as connection:
                require(connection.recv(12).startswith(b"RFB 003."), "VNC speaks RFB on localhost:5902")
            result = subprocess.run(["xdpyinfo"], env=child_env, capture_output=True, timeout=10)
            require(result.returncode == 0, "independent VNC Xauthority grants X11 access")

        static_tf = StaticTransformBroadcaster(node)
        map_tf = TransformStamped()
        map_tf.header.stamp = node.get_clock().now().to_msg()
        map_tf.header.frame_id = "map"
        map_tf.child_frame_id = "odom"
        map_tf.transform.rotation.w = 1.0
        static_tf.sendTransform(map_tf)
        dynamic_tf = TransformBroadcaster(node)
        odom_pub = node.create_publisher(Odometry, "/odom", 10)
        scan_pub = node.create_publisher(LaserScan, "/scan", 10)

        def publish_fixture():
            stamp = node.get_clock().now().to_msg()
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            transform.transform.rotation.w = 1.0
            dynamic_tf.sendTransform(transform)
            odom = Odometry()
            odom.header = transform.header
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = "base_link"
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = math.pi / 180.0
            scan.range_min = 0.2
            scan.range_max = 30.0
            scan.ranges = [float("inf")] * 361
            scan_pub.publish(scan)

        node.create_timer(0.05, publish_fixture)
        maps = []
        costmaps = []
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, "/map", maps.append, map_qos)
        node.create_subscription(OccupancyGrid, "/global_costmap/costmap", costmaps.append, map_qos)
        run_node("map", "nav2_map_server", "map_server", "-r", "__node:=map_server",
                 "--params-file", params, "-p", f"yaml_filename:={share / 'maps/arena.yaml'}")
        run_node("map-lifecycle", "nav2_lifecycle_manager", "lifecycle_manager",
                 "-r", "__node:=lifecycle_manager_test_map", "-p", "autostart:=true",
                 "-p", "node_names:=[map_server]")
        run_node("icp", "icp_probe", "direct_icp_probe", "--params-file",
                 share / "config/icp_localization.yaml", "-p",
                 f"map_path:={share / 'maps/arena.pcd'}", "-p", "auto_start:=false",
                 "-p", "publish_map_to_odom:=false")
        start("nav2", ["ros2", "launch", "nav2_bringup", "navigation_launch.py",
                       f"params_file:={params}", "use_composition:=False", "autostart:=true"])
        if not arguments.headless:
            rviz_binary = Path(get_package_prefix("rviz2")) / "lib/rviz2/rviz2"
            start("rviz", [str(rviz_binary), "-d", str(share / "rviz/nav2_vnc.rviz")])

        names = ["map_server", "controller_server", "smoother_server", "planner_server",
                 "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother"]
        clients = {name: node.create_client(GetState, f"/{name}/get_state") for name in names}
        futures = {}
        active = set()
        deadline = time.monotonic() + 55.0
        while time.monotonic() < deadline and len(active) < len(names):
            for label, process, _ in children:
                if process.poll() is not None:
                    raise RuntimeError(f"{label} exited early; inspect {logs}")
            if "process has died" in (logs / "nav2.log").read_text():
                raise RuntimeError(f"A Nav2 child exited early; inspect {logs / 'nav2.log'}")
            for name, client in clients.items():
                if name in active:
                    continue
                future = futures.get(name)
                if future is not None and future.done():
                    if future.result().current_state.id == 3:
                        active.add(name)
                        print(f"PASS: {name} lifecycle is active", flush=True)
                    futures.pop(name)
                elif future is None and client.service_is_ready():
                    futures[name] = client.call_async(GetState.Request())
            rclpy.spin_once(node, timeout_sec=0.1)
        require(len(active) == len(names), f"all 8 lifecycle nodes active; missing={set(names) - active}")
        until = time.monotonic() + 6.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
        require(bool(maps) and len(maps[-1].data) > 0, "recorded arena map is published")
        require(bool(costmaps), "configured global costmap publishes with synthetic TF")
        client = node.create_client(GetParameters, "/controller_server/get_parameters")
        require(client.wait_for_service(timeout_sec=3), "controller parameter service is available")
        request = GetParameters.Request(names=["FollowPath.plugin", "FollowPath.motion_model"])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        require(future.done() and [v.string_value for v in future.result().values] ==
                ["nav2_mppi_controller::MPPIController", "Omni"], "actual controller uses Omni MPPI")
        require("ICP ready:" in (logs / "icp.log").read_text(), "ICP loads the recorded PCD and initializes")
        if not arguments.headless:
            require("OpenGl version" in (logs / "rviz.log").read_text(),
                    "RViz initializes OpenGL on VNC :2")
            # Capture the dedicated test desktop for human visual inspection.
            subprocess.run(["/usr/bin/python3", "-c",
                            "from PyQt5.QtWidgets import QApplication; import sys; "
                            "app=QApplication([]); "
                            "sys.exit(0 if app.primaryScreen().grabWindow(0).save(sys.argv[1]) else 1)",
                            str(logs / "rviz-vnc.png")],
                           env=child_env, check=True, timeout=10)
        require(all(process.poll() is None for _, process, _ in children), "all tested processes remain alive")
        print("No driver/USB bridge or navigation goal was started; sensor and motion validation remain required.", flush=True)
    finally:
        # Signal only process groups that this invocation created.
        for _, process, _ in children:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        shutdown_deadline = time.monotonic() + 10.0
        for _, process, log in children:
            try:
                process.wait(timeout=max(0.1, shutdown_deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"Test children stopped. Logs retained at {logs}", flush=True)


if __name__ == "__main__":
    main()
