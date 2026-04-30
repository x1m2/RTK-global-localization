cmake_minimum_required(VERSION 3.8)
project(rtk_global_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(CODE "file(REMOVE_RECURSE \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/__pycache__\")")

install(
  PROGRAMS
    scripts/georeferenced_rtk_odometry.py
    scripts/imu_ned_to_enu.py
    scripts/map_point_to_navsat.py
    scripts/map_relative_heading_imu.py
    scripts/map_origin_recorder.py
    scripts/navsat_point_to_map.py
    scripts/navsat_fix_sanitizer.py
    scripts/odometry_low_pass_filter.py
    scripts/odometry_sanitizer.py
    scripts/pose_heading_to_imu.py
    scripts/simulated_start_pose.py
    scripts/static_heading_imu.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
