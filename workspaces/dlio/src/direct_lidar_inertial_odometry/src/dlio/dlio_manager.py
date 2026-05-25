#!/usr/bin/env python3
"""
DLIO Manager Node
Provides a ROS 2 service to start and stop the DLIO odometry and mapping nodes.

Usage:
  # Start DLIO:
  ros2 service call /dlio/enable std_srvs/srv/SetBool "{data: true}"
  # Stop DLIO:
  ros2 service call /dlio/enable std_srvs/srv/SetBool "{data: false}"
"""

import subprocess
import signal
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class DlioManager(Node):
    def __init__(self):
        super().__init__('dlio_manager')

        self._odom_proc = None
        self._map_proc = None
        self._is_running = False

        # Declare parameters (mirrors launch file args)
        self.declare_parameter('pointcloud_topic', '/points_raw_decoded')
        self.declare_parameter('imu_topic', '/livox/imu')

        self._srv = self.create_service(
            SetBool,
            '/dlio/enable',
            self._enable_callback
        )

        self.get_logger().info("DLIO Manager ready. Use /dlio/enable service to start/stop DLIO.")

    def _enable_callback(self, request, response):
        if request.data:
            if self._is_running:
                response.success = True
                response.message = "DLIO is already running."
                self.get_logger().warn("DLIO start requested but already running.")
                return response
            self._start_dlio()
            if self._is_running:
                response.success = True
                response.message = "DLIO started successfully."
            else:
                response.success = False
                response.message = "Failed to start DLIO."
        else:
            if not self._is_running:
                response.success = True
                response.message = "DLIO is not running."
                self.get_logger().warn("DLIO stop requested but already stopped.")
                return response
            self._stop_dlio()
            response.success = True
            response.message = "DLIO stopped."
        return response

    def _start_dlio(self):
        pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        self.get_logger().info(f"Starting DLIO (pointcloud: {pointcloud_topic}, imu: {imu_topic}) ...")

        # Find installed config/executable paths via environment
        install_share = self._find_install_share()

        if install_share is None:
            self.get_logger().error("Cannot find DLIO install/share directory. Is the package built and sourced?")
            return

        dlio_yaml = os.path.join(install_share, 'cfg', 'dlio.yaml')
        params_yaml = os.path.join(install_share, 'cfg', 'params.yaml')
        odom_exe = self._find_executable('dlio_odom_node')
        map_exe  = self._find_executable('dlio_map_node')

        if not odom_exe or not map_exe:
            self.get_logger().error("Cannot locate dlio_odom_node or dlio_map_node executables.")
            return

        odom_cmd = [
            odom_exe,
            '--ros-args',
            '--params-file', dlio_yaml,
            '--params-file', params_yaml,
            '-r', f'pointcloud:={pointcloud_topic}',
            '-r', f'imu:={imu_topic}',
            '-r', 'odom:=/lidar_odometry/pose',
            '-r', 'pose:=dlio/odom_node/pose',
            '-r', 'path:=dlio/odom_node/path',
            '-r', 'kf_pose:=dlio/odom_node/keyframes',
            '-r', 'kf_cloud:=dlio/odom_node/pointcloud/keyframe',
            '-r', 'deskewed:=/lidar_odometry/deskewed_scan_points',
        ]

        map_cmd = [
            map_exe,
            '--ros-args',
            '--params-file', dlio_yaml,
            '--params-file', params_yaml,
            '-r', 'keyframes:=dlio/odom_node/pointcloud/keyframe',
            '-r', 'odom:=/lidar_odometry/localmap_points',
        ]

        try:
            self._odom_proc = subprocess.Popen(odom_cmd, start_new_session=True)
            self._map_proc  = subprocess.Popen(map_cmd,  start_new_session=True)
            self._is_running = True
            self.get_logger().info(f"DLIO started (odom PID: {self._odom_proc.pid}, map PID: {self._map_proc.pid})")
        except Exception as e:
            self.get_logger().error(f"Failed to launch DLIO processes: {e}")
            self._is_running = False

    def _stop_dlio(self):
        self.get_logger().info("Stopping DLIO ...")
        for proc, name in [(self._odom_proc, 'odom'), (self._map_proc, 'map')]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=5)
                    self.get_logger().info(f"DLIO {name} node stopped (PID: {proc.pid})")
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f"DLIO {name} node did not stop cleanly, killing (PID: {proc.pid})")
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    proc.wait()
                except Exception as e:
                    self.get_logger().error(f"Error stopping DLIO {name} node: {e}")
        self._odom_proc = None
        self._map_proc = None
        self._is_running = False
        self.get_logger().info("DLIO stopped.")

    def _find_install_share(self):
        """Look for the installed share directory of this package."""
        ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
        for prefix in ament_prefix.split(':'):
            candidate = os.path.join(prefix, 'share', 'direct_lidar_inertial_odometry')
            if os.path.isdir(candidate):
                return candidate
        return None

    def _find_executable(self, name):
        """Look for the installed executable."""
        ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
        for prefix in ament_prefix.split(':'):
            candidate = os.path.join(prefix, 'lib', 'direct_lidar_inertial_odometry', name)
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
        return None

    def destroy_node(self):
        if self._is_running:
            self._stop_dlio()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DlioManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
