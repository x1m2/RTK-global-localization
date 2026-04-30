import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share_dir = get_package_share_directory("rtk_global_localization")

    default_map_origin_path = os.path.join(package_share_dir, "config", "map_origin.yaml")
    default_ekf_map_config = os.path.join(package_share_dir, "config", "ekf_map_origin.yaml")

    map_origin_path = LaunchConfiguration("map_origin_path")
    ekf_map_config = LaunchConfiguration("ekf_map_config")
    navsat_topic = LaunchConfiguration("navsat_topic")
    imu_ned_topic = LaunchConfiguration("imu_ned_topic")
    fastlio_odom_topic = LaunchConfiguration("fastlio_odom_topic")
    imu_yaw_correction = LaunchConfiguration("imu_yaw_correction")
    gps_smoothing_alpha = LaunchConfiguration("gps_smoothing_alpha")
    gps_smoothed_position_variance_floor = LaunchConfiguration(
        "gps_smoothed_position_variance_floor"
    )

    rtk_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share_dir, "launch", "rtk_global_localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "restamp_to_now": "false",
            "start_filters": "true",
            "start_pose_heading": "false",
            "start_static_heading": "false",
            "start_imu_converter": "true",
            "smooth_gps_odom": "true",
            "use_map_origin_localization": "true",
            "record_map_origin": "false",
            "publish_map_origin_debug_odom": "false",
            "map_origin_path": map_origin_path,
            "ekf_map_config": ekf_map_config,
            "publish_local_tf": "false",
            "publish_global_tf": "true",
            "publish_static_map_to_odom_tf": "false",
            "publish_odom_camera_init_tf": "false",
            "publish_gps_static_tf": "true",
            "navsat_topic": navsat_topic,
            "imu_ned_topic": imu_ned_topic,
            "fastlio_odom_topic": fastlio_odom_topic,
            "heading_topic": "/rtk/imu/enu",
            "global_heading_topic": "/rtk/navsat_heading/map_relative",
            "navsat_odom_topic": "/odometry/global",
            "gps_odom_topic": "/odometry/gps/smoothed",
            "gps_smoothed_odom_topic": "/odometry/gps/smoothed",
            "imu_yaw_correction": imu_yaw_correction,
            "gps_smoothing_alpha": gps_smoothing_alpha,
            "gps_smoothed_position_variance_floor": gps_smoothed_position_variance_floor,
            "global_fastlio_differential": "true",
            "fix_frame_id": "gps_link",
            # Vehicle calibration: body is the LiDAR center. The primary RTK
            # antenna is 5 cm behind, 5 cm left, and 3 cm above the LiDAR center.
            "gps_x": "-0.05",
            "gps_y": "0.05",
            "gps_z": "0.03",
            "gps_yaw": "0.0",
            "gps_pitch": "0.0",
            "gps_roll": "0.0",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_origin_path", default_value=default_map_origin_path),
            DeclareLaunchArgument("ekf_map_config", default_value=default_ekf_map_config),
            DeclareLaunchArgument("navsat_topic", default_value="/ap/navsat"),
            DeclareLaunchArgument("imu_ned_topic", default_value="/ap/imu/experimental/data"),
            DeclareLaunchArgument("fastlio_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument(
                "imu_yaw_correction",
                default_value="0.8776404167",
                description=(
                    "Yaw correction after NED->ENU conversion. Recalibrate this if "
                    "the live IMU heading does not align with the map heading."
                ),
            ),
            DeclareLaunchArgument("gps_smoothing_alpha", default_value="0.15"),
            DeclareLaunchArgument(
                "gps_smoothed_position_variance_floor",
                default_value="25.0",
                description=(
                    "Minimum covariance on smoothed GPS odometry. Lower this if RTK "
                    "should pull global localization harder; raise it if RTK is noisy."
                ),
            ),
            rtk_pipeline,
        ]
    )
