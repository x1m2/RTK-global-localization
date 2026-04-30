import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def _parse_map_origin_file(path: str) -> dict:
    values = {}
    with open(path, "r", encoding="utf-8") as origin_file:
        for raw_line in origin_file:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip().strip("'\"")
    return values


def _as_bool(value) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _build_navsat_transform(
    context,
    *,
    start_filters,
    use_map_origin_localization,
    wait_for_datum,
    map_origin_path,
    navsat_params,
    use_sim_time,
    navsat_yaw_offset,
    magnetic_declination,
    heading_topic,
    navsat_odom_topic,
):
    if not _as_bool(start_filters.perform(context)):
        return []

    parameters = [
        navsat_params,
        {
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "yaw_offset": ParameterValue(navsat_yaw_offset, value_type=float),
            "magnetic_declination_radians": ParameterValue(
                magnetic_declination, value_type=float
            ),
        },
    ]

    if _as_bool(use_map_origin_localization.perform(context)):
        resolved_origin_path = os.path.expanduser(map_origin_path.perform(context))
        if not os.path.exists(resolved_origin_path):
            raise RuntimeError(
                f"Fixed datum requested, but map origin file does not exist: {resolved_origin_path}"
            )

        origin = _parse_map_origin_file(resolved_origin_path)
        try:
            latitude = float(origin["latitude"])
            longitude = float(origin["longitude"])
            yaw_enu = float(origin["yaw_enu"])
        except KeyError as error:
            raise RuntimeError(
                f"Map origin file is missing required key {error!s}: {resolved_origin_path}"
            ) from error

        parameters[1]["wait_for_datum"] = True
        parameters[1]["datum"] = [latitude, longitude, yaw_enu]
    else:
        parameters[1]["wait_for_datum"] = ParameterValue(wait_for_datum, value_type=bool)

    return [
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=parameters,
            remappings=[
                ("gps/fix", "/rtk/fix"),
                ("imu", heading_topic),
                ("odometry/filtered", navsat_odom_topic),
                ("odometry/gps", "/odometry/gps"),
                ("gps/filtered", "/gps/filtered"),
            ],
        )
    ]


def generate_launch_description():
    package_share_dir = get_package_share_directory("rtk_global_localization")
    config_dir = os.path.join(package_share_dir, "config")

    use_sim_time = LaunchConfiguration("use_sim_time")
    restamp_to_now = LaunchConfiguration("restamp_to_now")
    start_filters = LaunchConfiguration("start_filters")
    start_pose_heading = LaunchConfiguration("start_pose_heading")
    start_static_heading = LaunchConfiguration("start_static_heading")
    start_imu_converter = LaunchConfiguration("start_imu_converter")
    smooth_gps_odom = LaunchConfiguration("smooth_gps_odom")
    use_map_origin_localization = LaunchConfiguration("use_map_origin_localization")
    record_map_origin = LaunchConfiguration("record_map_origin")
    publish_map_origin_debug_odom = LaunchConfiguration("publish_map_origin_debug_odom")
    ekf_autostart = LaunchConfiguration("ekf_autostart")
    publish_local_tf = LaunchConfiguration("publish_local_tf")
    publish_global_tf = LaunchConfiguration("publish_global_tf")
    publish_static_map_to_odom_tf = LaunchConfiguration("publish_static_map_to_odom_tf")
    publish_gps_static_tf = LaunchConfiguration("publish_gps_static_tf")
    publish_odom_camera_init_tf = LaunchConfiguration("publish_odom_camera_init_tf")

    navsat_topic = LaunchConfiguration("navsat_topic")
    pose_heading_topic = LaunchConfiguration("pose_heading_topic")
    imu_ned_topic = LaunchConfiguration("imu_ned_topic")
    imu_yaw_correction = LaunchConfiguration("imu_yaw_correction")
    fastlio_odom_topic = LaunchConfiguration("fastlio_odom_topic")
    heading_topic = LaunchConfiguration("heading_topic")
    global_heading_topic = LaunchConfiguration("global_heading_topic")
    navsat_odom_topic = LaunchConfiguration("navsat_odom_topic")
    gps_odom_topic = LaunchConfiguration("gps_odom_topic")
    gps_smoothed_odom_topic = LaunchConfiguration("gps_smoothed_odom_topic")
    map_origin_odom_topic = LaunchConfiguration("map_origin_odom_topic")

    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_link_frame = LaunchConfiguration("base_link_frame")
    fix_frame_id = LaunchConfiguration("fix_frame_id")
    odom_input_frame = LaunchConfiguration("odom_input_frame")
    odom_child_frame = LaunchConfiguration("odom_child_frame")

    gps_x = LaunchConfiguration("gps_x")
    gps_y = LaunchConfiguration("gps_y")
    gps_z = LaunchConfiguration("gps_z")
    gps_yaw = LaunchConfiguration("gps_yaw")
    gps_pitch = LaunchConfiguration("gps_pitch")
    gps_roll = LaunchConfiguration("gps_roll")

    heading_yaw_offset = LaunchConfiguration("heading_yaw_offset")
    initial_heading_yaw = LaunchConfiguration("initial_heading_yaw")
    global_initial_yaw = LaunchConfiguration("global_initial_yaw")
    map_origin_heading_yaw_offset = LaunchConfiguration("map_origin_heading_yaw_offset")
    navsat_yaw_offset = LaunchConfiguration("navsat_yaw_offset")
    magnetic_declination = LaunchConfiguration("magnetic_declination")
    wait_for_datum = LaunchConfiguration("wait_for_datum")
    map_origin_path = LaunchConfiguration("map_origin_path")
    map_origin_output_path = LaunchConfiguration("map_origin_output_path")
    map_origin_require_heading = LaunchConfiguration("map_origin_require_heading")
    map_origin_overwrite = LaunchConfiguration("map_origin_overwrite")
    map_origin_position_variance_floor = LaunchConfiguration(
        "map_origin_position_variance_floor"
    )
    map_origin_yaw_variance = LaunchConfiguration("map_origin_yaw_variance")
    gps_min_variance = LaunchConfiguration("gps_min_variance")
    gps_default_horizontal_variance = LaunchConfiguration("gps_default_horizontal_variance")
    gps_default_vertical_variance = LaunchConfiguration("gps_default_vertical_variance")
    gps_smoothing_alpha = LaunchConfiguration("gps_smoothing_alpha")
    gps_smoothed_position_variance_floor = LaunchConfiguration(
        "gps_smoothed_position_variance_floor"
    )
    global_fastlio_differential = LaunchConfiguration("global_fastlio_differential")
    reject_fastlio_outliers = LaunchConfiguration("reject_fastlio_outliers")
    fastlio_max_linear_speed = LaunchConfiguration("fastlio_max_linear_speed")
    fastlio_max_position_step = LaunchConfiguration("fastlio_max_position_step")
    fastlio_max_yaw_rate = LaunchConfiguration("fastlio_max_yaw_rate")

    ekf_odom_params = os.path.join(config_dir, "ekf_odom.yaml")
    default_ekf_map_params = os.path.join(config_dir, "ekf_map.yaml")
    ekf_map_params = LaunchConfiguration("ekf_map_config")
    navsat_params = os.path.join(config_dir, "navsat_transform.yaml")
    default_map_origin_path = os.path.join(config_dir, "map_origin.yaml")

    common_node_params = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
    }

    navsat_fix_sanitizer = Node(
        package="rtk_global_localization",
        executable="navsat_fix_sanitizer.py",
        name="navsat_fix_sanitizer",
        output="screen",
        parameters=[
            common_node_params,
            {
                "input_topic": navsat_topic,
                "output_topic": "/rtk/fix",
                "output_frame_id": fix_frame_id,
                "restamp_to_now": ParameterValue(restamp_to_now, value_type=bool),
                "minimum_variance": ParameterValue(gps_min_variance, value_type=float),
                "default_horizontal_variance": ParameterValue(
                    gps_default_horizontal_variance, value_type=float
                ),
                "default_vertical_variance": ParameterValue(
                    gps_default_vertical_variance, value_type=float
                ),
            },
        ],
    )

    pose_heading_to_imu = Node(
        condition=IfCondition(start_pose_heading),
        package="rtk_global_localization",
        executable="pose_heading_to_imu.py",
        name="pose_heading_to_imu",
        output="screen",
        parameters=[
            common_node_params,
            {
                "input_topic": pose_heading_topic,
                "output_topic": heading_topic,
                "output_frame_id": base_link_frame,
                "restamp_to_now": ParameterValue(restamp_to_now, value_type=bool),
                "yaw_offset": ParameterValue(heading_yaw_offset, value_type=float),
            },
        ],
    )

    static_heading_imu = Node(
        condition=IfCondition(start_static_heading),
        package="rtk_global_localization",
        executable="static_heading_imu.py",
        name="static_heading_imu",
        output="screen",
        parameters=[
            common_node_params,
            {
                "output_topic": heading_topic,
                "output_frame_id": base_link_frame,
                "yaw": ParameterValue(initial_heading_yaw, value_type=float),
            },
        ],
    )

    map_relative_heading_imu = Node(
        condition=IfCondition(use_map_origin_localization),
        package="rtk_global_localization",
        executable="map_relative_heading_imu.py",
        name="map_relative_heading_imu",
        output="screen",
        parameters=[
            common_node_params,
            {
                "input_topic": heading_topic,
                "output_topic": global_heading_topic,
                "map_origin_path": map_origin_path,
                "output_frame_id": base_link_frame,
            },
        ],
    )

    imu_ned_to_enu = Node(
        condition=IfCondition(start_imu_converter),
        package="rtk_global_localization",
        executable="imu_ned_to_enu.py",
        name="imu_ned_to_enu",
        output="screen",
        parameters=[
            common_node_params,
            {
                "input_topic": imu_ned_topic,
                "output_topic": "/rtk/imu/enu",
                "output_frame_id": base_link_frame,
                "restamp_to_now": ParameterValue(restamp_to_now, value_type=bool),
                "yaw_correction": ParameterValue(imu_yaw_correction, value_type=float),
            },
        ],
    )

    map_origin_recorder = Node(
        condition=IfCondition(record_map_origin),
        package="rtk_global_localization",
        executable="map_origin_recorder.py",
        name="map_origin_recorder",
        output="screen",
        parameters=[
            common_node_params,
            {
                "fix_topic": "/rtk/fix",
                "heading_topic": heading_topic,
                "output_path": map_origin_output_path,
                "frame_id": map_frame,
                "require_heading": ParameterValue(
                    map_origin_require_heading, value_type=bool
                ),
                "heading_yaw_offset": ParameterValue(
                    map_origin_heading_yaw_offset, value_type=float
                ),
                "overwrite": ParameterValue(map_origin_overwrite, value_type=bool),
            },
        ],
    )

    odometry_sanitizer = Node(
        package="rtk_global_localization",
        executable="odometry_sanitizer.py",
        name="fastlio_odometry_sanitizer",
        output="screen",
        parameters=[
            common_node_params,
            {
                "input_topic": fastlio_odom_topic,
                # Expose the sanitized FAST-LIO odometry directly as the local
                # odom topic used for RViz/debug and as the continuous motion
                # source for the global EKF.
                "output_topic": "/odometry/local",
                "output_frame_id": odom_input_frame,
                "output_child_frame_id": odom_child_frame,
                "restamp_to_now": ParameterValue(restamp_to_now, value_type=bool),
                "force_covariance": True,
                "xy_variance": 0.0025,
                "yaw_variance": 0.0025,
                "twist_linear_variance": 0.25,
                "twist_angular_variance": 0.10,
                "reject_kinematic_outliers": ParameterValue(
                    reject_fastlio_outliers, value_type=bool
                ),
                "max_linear_speed": ParameterValue(
                    fastlio_max_linear_speed, value_type=float
                ),
                "max_position_step": ParameterValue(
                    fastlio_max_position_step, value_type=float
                ),
                "max_yaw_rate": ParameterValue(fastlio_max_yaw_rate, value_type=float),
            },
        ],
    )

    gps_odom_smoother = Node(
        condition=IfCondition(smooth_gps_odom),
        package="rtk_global_localization",
        executable="odometry_low_pass_filter.py",
        name="gps_odometry_smoother",
        output="screen",
        parameters=[
            common_node_params,
            {
                "input_topic": "/odometry/gps",
                "output_topic": gps_smoothed_odom_topic,
                "alpha": ParameterValue(gps_smoothing_alpha, value_type=float),
                "position_variance_floor": ParameterValue(
                    gps_smoothed_position_variance_floor, value_type=float
                ),
            },
        ],
    )

    gps_static_tf = Node(
        condition=IfCondition(publish_gps_static_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_rtk_gps_tf",
        arguments=[
            gps_x,
            gps_y,
            gps_z,
            gps_yaw,
            gps_pitch,
            gps_roll,
            base_link_frame,
            fix_frame_id,
        ],
        output="screen",
    )

    georeferenced_rtk_odometry = Node(
        condition=IfCondition(publish_map_origin_debug_odom),
        package="rtk_global_localization",
        executable="georeferenced_rtk_odometry.py",
        name="georeferenced_rtk_odometry",
        output="screen",
        parameters=[
            common_node_params,
            {
                "fix_topic": "/rtk/fix",
                "heading_topic": heading_topic,
                "output_topic": map_origin_odom_topic,
                "map_origin_path": map_origin_path,
                "output_frame_id": map_frame,
                "child_frame_id": base_link_frame,
                "restamp_to_now": ParameterValue(restamp_to_now, value_type=bool),
                "require_heading": ParameterValue(
                    map_origin_require_heading, value_type=bool
                ),
                "heading_yaw_offset": ParameterValue(
                    map_origin_heading_yaw_offset, value_type=float
                ),
                "gps_x": ParameterValue(gps_x, value_type=float),
                "gps_y": ParameterValue(gps_y, value_type=float),
                "gps_z": ParameterValue(gps_z, value_type=float),
                "position_variance_floor": ParameterValue(
                    map_origin_position_variance_floor, value_type=float
                ),
                "yaw_variance": ParameterValue(map_origin_yaw_variance, value_type=float),
            },
        ],
    )

    odom_to_camera_init_tf = Node(
        condition=IfCondition(publish_odom_camera_init_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rtk_odom_to_camera_init_tf",
        arguments=["0", "0", "0", "0", "0", "0", odom_frame, "camera_init"],
        output="screen",
    )

    static_map_to_odom_tf = Node(
        condition=IfCondition(publish_static_map_to_odom_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rtk_static_map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", map_frame, odom_frame],
        output="screen",
    )

    ekf_odom = LifecycleNode(
        condition=IfCondition(start_filters),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        namespace="",
        output="screen",
        parameters=[
            ekf_odom_params,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_link_frame": base_link_frame,
                "base_link_frame_output": base_link_frame,
                "world_frame": odom_frame,
                "publish_tf": ParameterValue(publish_local_tf, value_type=bool),
            },
        ],
        remappings=[("odometry/filtered", "/odometry/local_ekf")],
    )

    ekf_map = LifecycleNode(
        condition=IfCondition(start_filters),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        namespace="",
        output="screen",
        parameters=[
            ekf_map_params,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_link_frame": base_link_frame,
                "base_link_frame_output": base_link_frame,
                "world_frame": map_frame,
                "publish_tf": ParameterValue(publish_global_tf, value_type=bool),
                "odom0_differential": ParameterValue(
                    global_fastlio_differential, value_type=bool
                ),
                "odom1": gps_odom_topic,
                "imu0": global_heading_topic,
                "initial_state": [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    global_initial_yaw,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            },
        ],
        remappings=[("odometry/filtered", "/odometry/global")],
    )

    ekf_lifecycle_manager = Node(
        condition=IfCondition(start_filters),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_rtk_ekf",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": ParameterValue(ekf_autostart, value_type=bool),
                "node_names": ["ekf_filter_node_odom", "ekf_filter_node_map"],
                "bond_timeout": 0.0,
            }
        ],
    )

    navsat_transform = OpaqueFunction(
        function=_build_navsat_transform,
        kwargs={
            "start_filters": start_filters,
            "use_map_origin_localization": use_map_origin_localization,
            "wait_for_datum": wait_for_datum,
            "map_origin_path": map_origin_path,
            "navsat_params": navsat_params,
            "use_sim_time": use_sim_time,
            "navsat_yaw_offset": navsat_yaw_offset,
            "magnetic_declination": magnetic_declination,
            "heading_topic": heading_topic,
            "navsat_odom_topic": navsat_odom_topic,
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "restamp_to_now",
                default_value="false",
                description="Restamp sanitized messages to node time. Use true for the provided bag.",
            ),
            DeclareLaunchArgument("start_filters", default_value="true"),
            DeclareLaunchArgument("start_pose_heading", default_value="false"),
            DeclareLaunchArgument("start_static_heading", default_value="true"),
            DeclareLaunchArgument("start_imu_converter", default_value="false"),
            DeclareLaunchArgument("smooth_gps_odom", default_value="false"),
            DeclareLaunchArgument(
                "use_map_origin_localization",
                default_value="false",
                description=(
                    "Use the saved map_origin.yaml as navsat_transform's fixed datum. "
                    "This removes first-GPS-frame drift while keeping the standard "
                    "robot_localization GPS pipeline."
                ),
            ),
            DeclareLaunchArgument(
                "record_map_origin",
                default_value="false",
                description="Record the current RTK fix and absolute heading as a map origin YAML.",
            ),
            DeclareLaunchArgument(
                "publish_map_origin_debug_odom",
                default_value="false",
                description=(
                    "Publish custom fixed-origin RTK odometry for debugging only. "
                    "Leave this false for the main robot_localization path."
                ),
            ),
            DeclareLaunchArgument(
                "ekf_map_config",
                default_value=default_ekf_map_params,
                description="Global EKF config file.",
            ),
            DeclareLaunchArgument(
                "ekf_autostart",
                default_value="true",
                description="Automatically configure and activate robot_localization EKF lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "publish_local_tf",
                default_value="false",
                description="Usually false because FAST-LIO already provides local TF.",
            ),
            DeclareLaunchArgument(
                "publish_global_tf",
                default_value="true",
                description=(
                    "Publish map->odom from the global EKF. Disable this for bag/RViz "
                    "debugging when you want the FAST-LIO local map to stay visually fixed."
                ),
            ),
            DeclareLaunchArgument(
                "publish_static_map_to_odom_tf",
                default_value="false",
                description=(
                    "Publish an identity static map->odom transform for visualization/debugging. "
                    "Do not enable together with dynamic publish_global_tf."
                ),
            ),
            DeclareLaunchArgument("navsat_topic", default_value="/ap/navsat"),
            DeclareLaunchArgument("pose_heading_topic", default_value="/ap/pose/filtered"),
            DeclareLaunchArgument("imu_ned_topic", default_value="/ap/imu/experimental/data"),
            DeclareLaunchArgument(
                "imu_yaw_correction",
                default_value="0.0",
                description=(
                    "Additional yaw correction applied after converting an IMU "
                    "orientation from NED to ENU."
                ),
            ),
            DeclareLaunchArgument("fastlio_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("heading_topic", default_value="/rtk/navsat_heading"),
            DeclareLaunchArgument(
                "global_heading_topic",
                default_value="/rtk/navsat_heading/map_relative",
                description=(
                    "Heading for the global EKF after converting absolute ENU yaw "
                    "into yaw relative to the saved map origin."
                ),
            ),
            DeclareLaunchArgument("gps_odom_topic", default_value="/odometry/gps"),
            DeclareLaunchArgument("gps_smoothed_odom_topic", default_value="/odometry/gps/smoothed"),
            DeclareLaunchArgument("map_origin_odom_topic", default_value="/rtk/map_origin/odom"),
            DeclareLaunchArgument(
                "navsat_odom_topic",
                default_value="/odometry/global",
                description=(
                    "Odometry reference used by navsat_transform. The official "
                    "dual_ekf_navsat pipeline feeds the global EKF output here."
                ),
            ),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_link_frame", default_value="body"),
            DeclareLaunchArgument(
                "fix_frame_id",
                default_value="body",
                description="Use body by default; switch to gps_link only when antenna TF is known.",
            ),
            DeclareLaunchArgument("odom_input_frame", default_value="odom"),
            DeclareLaunchArgument("odom_child_frame", default_value="body"),
            DeclareLaunchArgument("heading_yaw_offset", default_value="0.0"),
            DeclareLaunchArgument(
                "initial_heading_yaw",
                default_value="0.0",
                description="Static ROS ENU yaw in radians for navsat_transform initialization.",
            ),
            DeclareLaunchArgument(
                "global_initial_yaw",
                default_value="0.0",
                description=(
                    "Initial yaw for the global EKF state. Keep this at 0 when map and odom "
                    "are visualized with an identity static transform."
                ),
            ),
            DeclareLaunchArgument(
                "map_origin_path",
                default_value=default_map_origin_path,
                description=(
                    "YAML file with latitude, longitude, altitude, and yaw_enu for the "
                    "saved mapping origin."
                ),
            ),
            DeclareLaunchArgument(
                "map_origin_output_path",
                default_value="/tmp/rtk_map_origin.yaml",
                description="Output path used when record_map_origin=true.",
            ),
            DeclareLaunchArgument(
                "map_origin_require_heading",
                default_value="true",
                description="Require a dual-antenna/IMU heading before map-origin odometry is published.",
            ),
            DeclareLaunchArgument(
                "map_origin_overwrite",
                default_value="false",
                description="Allow map_origin_recorder to overwrite map_origin_output_path.",
            ),
            DeclareLaunchArgument(
                "map_origin_position_variance_floor",
                default_value="1.0",
                description=(
                    "Minimum X/Y/Z covariance used by the fixed-origin RTK odometry. "
                    "Raise this when RTK should initialize global pose but not pull hard."
                ),
            ),
            DeclareLaunchArgument(
                "map_origin_yaw_variance",
                default_value="0.0009",
                description="Yaw covariance for fixed-origin RTK odometry.",
            ),
            DeclareLaunchArgument(
                "map_origin_heading_yaw_offset",
                default_value="0.0",
                description="Additional yaw offset applied only by the map-origin recorder/odometry nodes.",
            ),
            DeclareLaunchArgument("navsat_yaw_offset", default_value="0.0"),
            DeclareLaunchArgument("magnetic_declination", default_value="0.0"),
            DeclareLaunchArgument("wait_for_datum", default_value="false"),
            DeclareLaunchArgument(
                "gps_min_variance",
                default_value="1.0e-6",
                description="Minimum covariance floor for valid RTK variances. Keep this tiny so good RTK covariances are preserved.",
            ),
            DeclareLaunchArgument(
                "gps_default_horizontal_variance",
                default_value="0.04",
                description="Fallback horizontal RTK variance in m^2 when the bag covariance is empty.",
            ),
            DeclareLaunchArgument(
                "gps_default_vertical_variance",
                default_value="0.25",
                description="Fallback vertical RTK variance in m^2 when the bag covariance is empty.",
            ),
            DeclareLaunchArgument(
                "gps_smoothing_alpha",
                default_value="0.15",
                description="Low-pass factor for /odometry/gps when smooth_gps_odom is true.",
            ),
            DeclareLaunchArgument(
                "gps_smoothed_position_variance_floor",
                default_value="0.0",
                description=(
                    "Minimum covariance used on /odometry/gps/smoothed. Raise this "
                    "when GPS should be a weak global correction instead of pulling "
                    "map->odom hard on every noisy RTK update."
                ),
            ),
            DeclareLaunchArgument(
                "global_fastlio_differential",
                default_value="true",
                description=(
                    "Use FAST-LIO pose differentially in the global EKF. Set false when "
                    "you want the global odometry to stay glued to the FAST-LIO local track "
                    "and use RTK only as a weak absolute correction."
                ),
            ),
            DeclareLaunchArgument(
                "reject_fastlio_outliers",
                default_value="false",
                description="Reject FAST-LIO odometry samples that violate vehicle kinematic limits.",
            ),
            DeclareLaunchArgument(
                "fastlio_max_linear_speed",
                default_value="8.0",
                description="Maximum accepted FAST-LIO translational speed in m/s.",
            ),
            DeclareLaunchArgument(
                "fastlio_max_position_step",
                default_value="3.0",
                description="Maximum accepted FAST-LIO position step between messages in m.",
            ),
            DeclareLaunchArgument(
                "fastlio_max_yaw_rate",
                default_value="2.5",
                description="Maximum accepted FAST-LIO yaw rate in rad/s.",
            ),
            DeclareLaunchArgument("publish_odom_camera_init_tf", default_value="false"),
            DeclareLaunchArgument("publish_gps_static_tf", default_value="false"),
            DeclareLaunchArgument("gps_x", default_value="0.0"),
            DeclareLaunchArgument("gps_y", default_value="0.0"),
            DeclareLaunchArgument("gps_z", default_value="0.0"),
            DeclareLaunchArgument("gps_yaw", default_value="0.0"),
            DeclareLaunchArgument("gps_pitch", default_value="0.0"),
            DeclareLaunchArgument("gps_roll", default_value="0.0"),
            navsat_fix_sanitizer,
            pose_heading_to_imu,
            static_heading_imu,
            map_relative_heading_imu,
            imu_ned_to_enu,
            map_origin_recorder,
            odometry_sanitizer,
            gps_odom_smoother,
            gps_static_tf,
            georeferenced_rtk_odometry,
            odom_to_camera_init_tf,
            static_map_to_odom_tf,
            ekf_odom,
            ekf_map,
            ekf_lifecycle_manager,
            navsat_transform,
        ]
    )
