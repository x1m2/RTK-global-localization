import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _guess_workspace_root(package_share_dir: str) -> str:
    install_marker = os.path.sep + "install" + os.path.sep
    src_marker = os.path.sep + "src" + os.path.sep

    if install_marker in package_share_dir:
        return package_share_dir.split(install_marker)[0]
    if src_marker in package_share_dir:
        return package_share_dir.split(src_marker)[0]
    return os.path.abspath(os.path.join(package_share_dir, "..", "..", ".."))


def generate_launch_description():
    package_share_dir = get_package_share_directory("rtk_global_localization")
    fast_lio_share_dir = get_package_share_directory("fast_lio")
    workspace_root = _guess_workspace_root(package_share_dir)

    default_bag = os.path.join(workspace_root, "bags", "ap_lidar_all_20260324_175502")
    default_fastlio_config = os.path.join(fast_lio_share_dir, "config", "mid360.yaml")
    default_map_origin_path = os.path.join(package_share_dir, "config", "map_origin.yaml")
    default_ekf_map_config = os.path.join(package_share_dir, "config", "ekf_map_origin.yaml")
    default_map_yaml = os.path.join(workspace_root, "maps", "menkou.yaml")

    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    bag_start_offset = LaunchConfiguration("bag_start_offset")
    play_bag = LaunchConfiguration("play_bag")
    start_fastlio = LaunchConfiguration("start_fastlio")
    start_pipeline = LaunchConfiguration("start_pipeline")
    start_odom_camera_init_tf = LaunchConfiguration("start_odom_camera_init_tf")
    start_map_server = LaunchConfiguration("start_map_server")
    start_map_point_converter = LaunchConfiguration("start_map_point_converter")
    map_yaml = LaunchConfiguration("map_yaml")
    map_topic = LaunchConfiguration("map_topic")
    map_frame = LaunchConfiguration("map_frame")
    bag_start_delay = LaunchConfiguration("bag_start_delay")
    fastlio_config = LaunchConfiguration("fastlio_config")
    initial_heading_yaw = LaunchConfiguration("initial_heading_yaw")
    imu_yaw_correction = LaunchConfiguration("imu_yaw_correction")
    global_initial_yaw = LaunchConfiguration("global_initial_yaw")
    navsat_odom_topic = LaunchConfiguration("navsat_odom_topic")
    gps_odom_topic = LaunchConfiguration("gps_odom_topic")
    use_map_origin_localization = LaunchConfiguration("use_map_origin_localization")
    record_map_origin = LaunchConfiguration("record_map_origin")
    publish_map_origin_debug_odom = LaunchConfiguration("publish_map_origin_debug_odom")
    map_origin_path = LaunchConfiguration("map_origin_path")
    map_origin_output_path = LaunchConfiguration("map_origin_output_path")
    map_origin_require_heading = LaunchConfiguration("map_origin_require_heading")
    map_origin_overwrite = LaunchConfiguration("map_origin_overwrite")
    map_origin_odom_topic = LaunchConfiguration("map_origin_odom_topic")
    map_origin_position_variance_floor = LaunchConfiguration(
        "map_origin_position_variance_floor"
    )
    map_origin_yaw_variance = LaunchConfiguration("map_origin_yaw_variance")
    ekf_map_config = LaunchConfiguration("ekf_map_config")
    gps_min_variance = LaunchConfiguration("gps_min_variance")
    gps_default_horizontal_variance = LaunchConfiguration("gps_default_horizontal_variance")
    gps_default_vertical_variance = LaunchConfiguration("gps_default_vertical_variance")
    gps_smoothing_alpha = LaunchConfiguration("gps_smoothing_alpha")
    gps_smoothed_position_variance_floor = LaunchConfiguration(
        "gps_smoothed_position_variance_floor"
    )
    global_fastlio_differential = LaunchConfiguration("global_fastlio_differential")
    publish_global_tf = LaunchConfiguration("publish_global_tf")
    publish_static_map_to_odom_tf = LaunchConfiguration("publish_static_map_to_odom_tf")
    reject_fastlio_outliers = LaunchConfiguration("reject_fastlio_outliers")
    fastlio_max_linear_speed = LaunchConfiguration("fastlio_max_linear_speed")
    fastlio_max_position_step = LaunchConfiguration("fastlio_max_position_step")
    fastlio_max_yaw_rate = LaunchConfiguration("fastlio_max_yaw_rate")

    fastlio_node = Node(
        condition=IfCondition(start_fastlio),
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[
            fastlio_config,
            {"use_sim_time": ParameterValue(True, value_type=bool)},
        ],
    )

    odom_to_camera_init_tf = Node(
        condition=IfCondition(start_odom_camera_init_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rtk_odom_to_camera_init_tf",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
        output="screen",
    )

    map_server = Node(
        condition=IfCondition(start_map_server),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(True, value_type=bool),
                "yaml_filename": map_yaml,
                "topic_name": map_topic,
                "frame_id": map_frame,
            }
        ],
    )

    map_lifecycle_manager = Node(
        condition=IfCondition(start_map_server),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_bag_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(True, value_type=bool),
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    map_point_converter = Node(
        condition=IfCondition(start_map_point_converter),
        package="rtk_global_localization",
        executable="map_point_to_navsat.py",
        name="map_point_to_navsat",
        output="screen",
        parameters=[
            {
                "input_topic": "/clicked_point",
                "output_topic": "/clicked_point/navsat",
                "map_origin_path": map_origin_path,
                "map_frame": map_frame,
                "output_frame_id": map_frame,
            }
        ],
    )

    rtk_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share_dir, "launch", "rtk_global_localization.launch.py")
        ),
        condition=IfCondition(start_pipeline),
        launch_arguments={
            "use_sim_time": "true",
            "restamp_to_now": "true",
            "start_filters": "true",
            "start_pose_heading": "false",
            "start_static_heading": "false",
            "start_imu_converter": "true",
            "smooth_gps_odom": "true",
            "use_map_origin_localization": use_map_origin_localization,
            "record_map_origin": record_map_origin,
            "publish_map_origin_debug_odom": publish_map_origin_debug_odom,
            "map_origin_path": map_origin_path,
            "map_origin_output_path": map_origin_output_path,
            "map_origin_require_heading": map_origin_require_heading,
            "map_origin_overwrite": map_origin_overwrite,
            "map_origin_position_variance_floor": map_origin_position_variance_floor,
            "map_origin_yaw_variance": map_origin_yaw_variance,
            "ekf_map_config": ekf_map_config,
            "map_origin_odom_topic": map_origin_odom_topic,
            "publish_local_tf": "false",
            "publish_global_tf": publish_global_tf,
            "publish_static_map_to_odom_tf": publish_static_map_to_odom_tf,
            "publish_odom_camera_init_tf": "false",
            "publish_gps_static_tf": "true",
            "fix_frame_id": "gps_link",
            "heading_topic": "/rtk/imu/enu",
            "initial_heading_yaw": initial_heading_yaw,
            "imu_yaw_correction": imu_yaw_correction,
            "global_initial_yaw": global_initial_yaw,
            "navsat_odom_topic": navsat_odom_topic,
            # The bag RTK has visible lateral steps. Feed the map EKF the
            # low-pass GPS odometry so RTK acts as a weak global correction
            # instead of jerking map->odom on each raw GPS update.
            "gps_odom_topic": gps_odom_topic,
            "gps_smoothed_odom_topic": "/odometry/gps/smoothed",
            "gps_min_variance": gps_min_variance,
            "gps_default_horizontal_variance": gps_default_horizontal_variance,
            "gps_default_vertical_variance": gps_default_vertical_variance,
            "gps_smoothing_alpha": gps_smoothing_alpha,
            "gps_smoothed_position_variance_floor": gps_smoothed_position_variance_floor,
            "global_fastlio_differential": global_fastlio_differential,
            "reject_fastlio_outliers": reject_fastlio_outliers,
            "fastlio_max_linear_speed": fastlio_max_linear_speed,
            "fastlio_max_position_step": fastlio_max_position_step,
            "fastlio_max_yaw_rate": fastlio_max_yaw_rate,
        }.items(),
    )

    bag_play = TimerAction(
        condition=IfCondition(play_bag),
        period=bag_start_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag_path,
                    "--clock",
                    "--rate",
                    bag_rate,
                    "--start-offset",
                    bag_start_offset,
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_path", default_value=default_bag),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument(
                "bag_start_offset",
                default_value="0.0",
                description="Start rosbag playback at this offset in seconds.",
            ),
            DeclareLaunchArgument("bag_start_delay", default_value="4.0"),
            DeclareLaunchArgument("play_bag", default_value="true"),
            DeclareLaunchArgument("start_fastlio", default_value="true"),
            DeclareLaunchArgument("start_pipeline", default_value="true"),
            DeclareLaunchArgument("start_odom_camera_init_tf", default_value="true"),
            DeclareLaunchArgument(
                "start_map_server",
                default_value="true",
                description=(
                    "Publish the bag's fixed 2D occupancy grid for RViz global localization verification."
                ),
            ),
            DeclareLaunchArgument(
                "start_map_point_converter",
                default_value="true",
                description=(
                    "Convert RViz /clicked_point map coordinates to WGS84 latitude/longitude."
                ),
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=default_map_yaml,
                description="2D occupancy-grid map YAML used by the bag verification view.",
            ),
            DeclareLaunchArgument(
                "map_topic",
                default_value="map",
                description="Topic name for the fixed 2D occupancy grid.",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Frame id stamped on the fixed 2D occupancy grid.",
            ),
            DeclareLaunchArgument("fastlio_config", default_value=default_fastlio_config),
            DeclareLaunchArgument(
                "initial_heading_yaw",
                default_value="2.443460953",
                description=(
                    "Initial ROS ENU yaw for this Chongqing bag. The vehicle initially "
                    "faced compass 310 deg, which is ROS ENU yaw 140 deg."
                ),
            ),
            DeclareLaunchArgument(
                "imu_yaw_correction",
                default_value="0.8776404167",
                description=(
                    "Bag-specific yaw correction applied after NED->ENU conversion of "
                    "/ap/imu/experimental/data so the first converted IMU yaw matches "
                    "the saved map-origin yaw."
                ),
            ),
            DeclareLaunchArgument(
                "global_initial_yaw",
                default_value="0.0",
                description=(
                    "Initial yaw for /odometry/global display. Keep 0.0 for the bag debug "
                    "mode where map->odom is a static identity transform."
                ),
            ),
            DeclareLaunchArgument(
                "navsat_odom_topic",
                default_value="/odometry/global",
                description=(
                    "Use the global EKF output as navsat_transform reference, matching "
                    "robot_localization's dual_ekf_navsat example."
                ),
            ),
            DeclareLaunchArgument(
                "gps_odom_topic",
                default_value="/odometry/gps/smoothed",
                description=(
                    "GPS odometry topic fused by the global EKF. Override this to "
                    "/no_gps for an A/B run that proves whether GPS is being fused."
                ),
            ),
            DeclareLaunchArgument(
                "use_map_origin_localization",
                default_value="true",
                description=(
                    "Use the saved mapping origin as navsat_transform's fixed datum. "
                    "This is what makes arbitrary bag start offsets deterministic."
                ),
            ),
            DeclareLaunchArgument(
                "record_map_origin",
                default_value="false",
                description="Record a new map origin while playing/running.",
            ),
            DeclareLaunchArgument(
                "publish_map_origin_debug_odom",
                default_value="false",
                description="Publish custom fixed-origin RTK odometry only for debugging.",
            ),
            DeclareLaunchArgument("map_origin_path", default_value=default_map_origin_path),
            DeclareLaunchArgument(
                "map_origin_output_path",
                default_value="/tmp/rtk_map_origin.yaml",
            ),
            DeclareLaunchArgument("map_origin_require_heading", default_value="true"),
            DeclareLaunchArgument("map_origin_overwrite", default_value="false"),
            DeclareLaunchArgument(
                "map_origin_odom_topic",
                default_value="/rtk/map_origin/odom",
                description=(
                    "Debug topic used only when publish_map_origin_debug_odom=true."
                ),
            ),
            DeclareLaunchArgument(
                "map_origin_position_variance_floor",
                default_value="1.0",
                description=(
                    "Minimum covariance for the optional fixed-origin debug odometry."
                ),
            ),
            DeclareLaunchArgument("map_origin_yaw_variance", default_value="0.0009"),
            DeclareLaunchArgument("ekf_map_config", default_value=default_ekf_map_config),
            DeclareLaunchArgument(
                "gps_min_variance",
                default_value="0.5",
                description=(
                    "Temporary covariance floor for this bag. We intentionally down-weight "
                    "GPS until the RTK antenna offset is modeled."
                ),
            ),
            DeclareLaunchArgument(
                "gps_default_horizontal_variance",
                default_value="1.0",
                description=(
                    "Fallback horizontal variance for bag messages that have missing GPS covariance."
                ),
            ),
            DeclareLaunchArgument(
                "gps_default_vertical_variance",
                default_value="1.0",
                description=(
                    "Fallback vertical variance for bag messages that have missing GPS covariance."
                ),
            ),
            DeclareLaunchArgument(
                "gps_smoothing_alpha",
                default_value="0.15",
                description=(
                    "Low-pass factor for the stair-stepped GPS odometry in this bag. "
                    "Keep this responsive; the EKF weight is controlled by covariance."
                ),
            ),
            DeclareLaunchArgument(
                "gps_smoothed_position_variance_floor",
                default_value="25.0",
                description=(
                    "Minimum covariance used by /odometry/gps/smoothed in the bag demo. "
                    "This makes RTK a gentle global correction while FAST-LIO provides "
                    "the smooth local motion."
                ),
            ),
            DeclareLaunchArgument(
                "global_fastlio_differential",
                default_value="true",
                description=(
                    "Use FAST-LIO/local odometry as a relative-motion source in the "
                    "global EKF. This matches our FAST-LIO setup better than fusing "
                    "odom-frame pose as an absolute map-frame measurement."
                ),
            ),
            DeclareLaunchArgument(
                "publish_global_tf",
                default_value="true",
                description=(
                    "Publish dynamic map->odom from the global EKF. This lets /odometry/local "
                    "and /odometry/gps display in the fixed map frame at the saved map-origin pose."
                ),
            ),
            DeclareLaunchArgument(
                "publish_static_map_to_odom_tf",
                default_value="false",
                description=(
                    "Publish identity static map->odom only for pure local debugging. Do not "
                    "enable together with publish_global_tf."
                ),
            ),
            DeclareLaunchArgument(
                "reject_fastlio_outliers",
                default_value="true",
                description="Drop FAST-LIO odometry samples that violate vehicle kinematic limits.",
            ),
            DeclareLaunchArgument(
                "fastlio_max_linear_speed",
                default_value="6.0",
                description="Maximum physically plausible vehicle speed in m/s for this bag.",
            ),
            DeclareLaunchArgument(
                "fastlio_max_position_step",
                default_value="2.0",
                description="Maximum FAST-LIO position step between consecutive odometry messages.",
            ),
            DeclareLaunchArgument(
                "fastlio_max_yaw_rate",
                default_value="2.0",
                description="Maximum physically plausible yaw rate in rad/s for this bag.",
            ),
            fastlio_node,
            odom_to_camera_init_tf,
            map_server,
            map_lifecycle_manager,
            map_point_converter,
            rtk_pipeline,
            bag_play,
        ]
    )
