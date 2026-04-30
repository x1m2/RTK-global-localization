#!/usr/bin/env bash
set -euo pipefail

# Cleanup helper for duplicate localization processes ("ghost nodes").
pkill -f "ros2 launch fast_lio dual_ekf_rtk.launch.py" || true
pkill -f "fastlio_mapping" || true
pkill -f "robot_localization.*ekf_node" || true
pkill -f "navsat_transform_node" || true
pkill -f "imu_ned_to_enu.py" || true
pkill -f "map_anchor_injector.py" || true
pkill -f "ros2 bag play" || true

echo "cleanup_dual_ekf.sh: stale localization processes cleared."
