#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"

# DDS selection:
# - If user explicitly sets RMW_IMPLEMENTATION, respect it.
# - Otherwise prefer CycloneDDS, and auto-fallback to FastDDS if Cyclone plugin is missing.
if [[ -z "${RMW_IMPLEMENTATION:-}" ]]; then
  if [[ -f "/opt/ros/humble/lib/librmw_cyclonedds_cpp.so" ]]; then
    export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
  else
    export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
    echo "[WARN] librmw_cyclonedds_cpp.so not found, fallback to rmw_fastrtps_cpp"
  fi
fi

# Clean up stale nodes from previous crashed/forced runs to avoid duplicate graphs.
if [[ "${NAV2_SKIP_CLEANUP:-0}" != "1" ]]; then
  kill_by_pattern() {
    local pattern="$1"
    local pids
    pids="$(pgrep -f "${pattern}" || true)"
    if [[ -n "${pids}" ]]; then
      kill ${pids} 2>/dev/null || true
      sleep 0.2
      pids="$(pgrep -f "${pattern}" || true)"
      if [[ -n "${pids}" ]]; then
        kill -9 ${pids} 2>/dev/null || true
      fi
    fi
  }

  kill_by_pattern "__node:=lifecycle_manager_map"
  kill_by_pattern "__node:=lifecycle_manager_navigation"
  # Match both package-style and absolute-file launch invocations.
  kill_by_pattern "ros2 launch .*nav2_fastlio_rviz.launch.py"
  kill_by_pattern "__node:=map_server"
  kill_by_pattern "__node:=controller_server"
  kill_by_pattern "__node:=planner_server"
  kill_by_pattern "__node:=behavior_server"
  kill_by_pattern "__node:=bt_navigator"
  kill_by_pattern "__node:=waypoint_follower"
  kill_by_pattern "__node:=velocity_smoother"
  kill_by_pattern "__node:=smoother_server"
  kill_by_pattern "__node:=fake_base_odom"
  kill_by_pattern "__node:=map_to_odom_tf"
  kill_by_pattern "__node:=odom_to_camera_init_tf"
  kill_by_pattern "__node:=body_to_level_tf"
  kill_by_pattern "__node:=robot_state_publisher"
  kill_by_pattern "__node:=pointcloud_to_laserscan"
  kill_by_pattern "__node:=pointcloud_to_laserscan_local"
  kill_by_pattern "__node:=cmd_vel_to_ap_bridge"
  kill_by_pattern "__node:=nav2_container"
  kill_by_pattern "__node:=livox_lidar_publisher"
  kill_by_pattern "livox_ros_driver2_node"
  kill_by_pattern "fastlio_mapping"
  kill_by_pattern "__node:=laser_mapping"
  kill_by_pattern "nav2_default_view.rviz"
  kill_by_pattern "rviz2"

  sleep 1
fi

launch_args=("$@")
has_use_rviz=0
use_hardware_true=0

for arg in "${launch_args[@]}"; do
  case "${arg}" in
    use_rviz:=*)
      has_use_rviz=1
      ;;
    use_hardware:=*)
      hw_value="${arg#use_hardware:=}"
      hw_lower="${hw_value,,}"
      if [[ "${hw_lower}" == "true" ]]; then
        use_hardware_true=1
      fi
      ;;
  esac
done

# Hardware mode should be headless by default to avoid RViz overloading the IPC.
if [[ ${use_hardware_true} -eq 1 && ${has_use_rviz} -eq 0 ]]; then
  launch_args+=("use_rviz:=false")
  echo "[INFO] use_hardware:=true detected; defaulting to use_rviz:=false."
fi

source_launch_file="${ROOT_DIR}/src/model/launch/nav2_fastlio_rviz.launch.py"
if [[ -f "${source_launch_file}" ]]; then
  exec ros2 launch "${source_launch_file}" "${launch_args[@]}"
fi

exec ros2 launch nav_simplified_model nav2_fastlio_rviz.launch.py "${launch_args[@]}"
