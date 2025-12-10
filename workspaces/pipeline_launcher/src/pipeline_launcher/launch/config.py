#!/usr/bin/env python3
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
"""
Pipeline Launcher Configuration Module.

Centralized configuration for all pipeline launch files. Edit values here
to change topic names, timing delays, and launch arguments across all
pipeline variants.
"""

from dataclasses import dataclass, field
from typing import Dict


# =============================================================================
# Package Names
# =============================================================================
PACKAGE_DLIO = "direct_lidar_inertial_odometry"
PACKAGE_OPEN3D_SLAM = "open3d_slam_ros"
PACKAGE_GO2_SIMULATOR = "go2_simulator"
PACKAGE_FAR_PLANNER = "far_planner"
PACKAGE_FOXGLOVE_BRIDGE = "foxglove_bridge"
PACKAGE_FAST_LIO = "fast_lio"


# =============================================================================
# Launch File Paths (relative to package share directory)
# =============================================================================
LAUNCH_DLIO = "/launch/dlio.launch.py"
LAUNCH_OPEN3D_SLAM = "/launch/open3d_launch.py"
LAUNCH_GO2_SIMULATOR = "/launch/system_real_robot.launch"
LAUNCH_FAR_PLANNER = "/launch/far_planner.launch"
LAUNCH_FOXGLOVE_BRIDGE = "/launch/foxglove_bridge_launch.xml"
LAUNCH_FAST_LIO = "/launch/mapping_mid360.launch.py"


# =============================================================================
# Topic Configuration
# =============================================================================
@dataclass(frozen=True)
class TopicConfig:
    """Sensor topic configuration for different robot setups."""

    pointcloud: str
    imu: str
    odometry: str = ""  # Optional odometry topic


# Predefined topic configurations
TOPICS_LIVOX = TopicConfig(
    pointcloud="/livox/lidar",
    imu="/livox/imu",
    odometry="",
)

TOPICS_VELODYNE = TopicConfig(
    pointcloud="/velodyne_points",
    imu="/imu/data",
    odometry="",
)

TOPICS_DLIO_OUTPUT = TopicConfig(
    pointcloud="/registered_scan",
    imu="/livox/imu",
    odometry="/state_estimation",
)


# =============================================================================
# Timing Configuration (in seconds)
# =============================================================================
@dataclass(frozen=True)
class TimingConfig:
    """Launch timing delays for pipeline components."""

    foxglove_bridge: float = 0.0
    dlio: float = 5.0
    open3d_slam: float = 5.0
    go2_simulator: float = 5.0
    far_planner: float = 5.0


# Default timing configuration
DEFAULT_TIMING = TimingConfig()

# Quick start timing (minimal delays for testing)
QUICK_TIMING = TimingConfig(
    foxglove_bridge=0.0,
    dlio=1.0,
    open3d_slam=1.0,
    go2_simulator=1.0,
    far_planner=1.0,
)


# =============================================================================
# Launch Argument Defaults
# =============================================================================
@dataclass
class DlioArgs:
    """DLIO launch arguments."""

    rviz: str = "false"
    pointcloud_topic: str = TOPICS_LIVOX.pointcloud
    imu_topic: str = TOPICS_LIVOX.imu

    def to_dict(self) -> Dict[str, str]:
        """Convert to launch arguments dictionary."""
        return {
            "rviz": self.rviz,
            "pointcloud_topic": self.pointcloud_topic,
            "imu_topic": self.imu_topic,
        }


@dataclass
class Open3dSlamArgs:
    """Open3D SLAM launch arguments."""

    use_sim_time: str = "false"
    launch_rviz: str = "false"
    cloud_topic: str = TOPICS_DLIO_OUTPUT.pointcloud
    odometry_topic: str = TOPICS_DLIO_OUTPUT.odometry

    def to_dict(self) -> Dict[str, str]:
        """Convert to launch arguments dictionary."""
        return {
            "use_sim_time": self.use_sim_time,
            "launch_rviz": self.launch_rviz,
            "cloud_topic": self.cloud_topic,
            "odometry_topic": self.odometry_topic,
        }


@dataclass
class FarPlannerArgs:
    """Far Planner launch arguments."""

    use_sim_time: str = "false"

    def to_dict(self) -> Dict[str, str]:
        """Convert to launch arguments dictionary."""
        return {"use_sim_time": self.use_sim_time}


@dataclass
class FastLioArgs:
    """Fast-LIO launch arguments."""

    rviz: str = "false"

    def to_dict(self) -> Dict[str, str]:
        """Convert to launch arguments dictionary."""
        return {"rviz": self.rviz}
