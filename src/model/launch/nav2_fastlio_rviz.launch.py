import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _guess_workspace_root(model_share_dir: str) -> str:
    install_marker = os.path.sep + "install" + os.path.sep
    src_marker = os.path.sep + "src" + os.path.sep

    if install_marker in model_share_dir:
        return model_share_dir.split(install_marker)[0]
    if src_marker in model_share_dir:
        return model_share_dir.split(src_marker)[0]
    return os.path.abspath(os.path.join(model_share_dir, "..", "..", ".."))


def generate_launch_description():
    model_share_dir = get_package_share_directory("nav_simplified_model")
    nav2_share_dir = get_package_share_directory("nav2_bringup")
    fast_lio_share_dir = get_package_share_directory("fast_lio")
    livox_share_dir = get_package_share_directory("livox_ros_driver2")

    workspace_root = _guess_workspace_root(model_share_dir)
    default_map = os.path.join(workspace_root, "src", "maps", "changqu_new.yaml")
    stamp_relay_script = os.path.join(
        workspace_root,
        "src",
        "model",
        "scripts",
        "header_stamp_relay.py",
    )
    default_params_rviz = os.path.join(
        workspace_root,
        "src",
        "navigation2",
        "nav2_bringup",
        "params",
        "nav2_params.yaml",
    )
    default_params_hardware = os.path.join(
        workspace_root,
        "src",
        "navigation2",
        "nav2_bringup",
        "params",
        "nav2_params_fastlio.yaml",
    )
    default_urdf = os.path.join(model_share_dir, "urdf", "nav_simplified_model.urdf")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    hardware_params_file = LaunchConfiguration("hardware_params_file")
    urdf_file = LaunchConfiguration("urdf")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    fastlio_config = LaunchConfiguration("fastlio_config")
    use_hardware = LaunchConfiguration("use_hardware")
    start_livox_driver = LaunchConfiguration("start_livox_driver")
    livox_publish_freq = LaunchConfiguration("livox_publish_freq")
    livox_frame_id = LaunchConfiguration("livox_frame_id")
    hardware_nav2_start_delay_sec = LaunchConfiguration("hardware_nav2_start_delay_sec")
    ap_cmd_vel_topic = LaunchConfiguration("ap_cmd_vel_topic")
    nav2_cmd_vel_topic = LaunchConfiguration("nav2_cmd_vel_topic")
    lidar_pitch_compensation = LaunchConfiguration("lidar_pitch_compensation")
    local_scan_min_height = LaunchConfiguration("local_scan_min_height")
    local_scan_max_height = LaunchConfiguration("local_scan_max_height")
    local_scan_ground_percentile = LaunchConfiguration("local_scan_ground_percentile")
    local_scan_ground_margin = LaunchConfiguration("local_scan_ground_margin")
    local_scan_min_obstacle_rel_z = LaunchConfiguration("local_scan_min_obstacle_rel_z")
    local_scan_min_block_points = LaunchConfiguration("local_scan_min_block_points")
    local_scan_min_obstacle_points = LaunchConfiguration("local_scan_min_obstacle_points")
    local_scan_terrain_max_rel_z = LaunchConfiguration("local_scan_terrain_max_rel_z")
    local_scan_terrain_dis_ratio_z = LaunchConfiguration("local_scan_terrain_dis_ratio_z")
    local_scan_planar_size = LaunchConfiguration("local_scan_planar_size")
    local_scan_neighbor_spread = LaunchConfiguration("local_scan_neighbor_spread")
    local_scan_target_frame = LaunchConfiguration("local_scan_target_frame")
    local_scan_obstacle_range_slope = LaunchConfiguration("local_scan_obstacle_range_slope")
    local_scan_range_min = LaunchConfiguration("local_scan_range_min")
    local_scan_range_max = LaunchConfiguration("local_scan_range_max")
    local_scan_terrain_min_rel_z = LaunchConfiguration("local_scan_terrain_min_rel_z")
    local_scan_min_ground_points = LaunchConfiguration("local_scan_min_ground_points")
    local_scan_vehicle_height = LaunchConfiguration("local_scan_vehicle_height")
    enable_waypoint_loop = LaunchConfiguration("enable_waypoint_loop")
    waypoint_loop_topic = LaunchConfiguration("waypoint_loop_topic")
    waypoint_loop_topic_type = LaunchConfiguration("waypoint_loop_topic_type")
    waypoint_loop_waypoints = LaunchConfiguration("waypoint_loop_waypoints")
    waypoint_loop_forever = LaunchConfiguration("waypoint_loop_forever")
    waypoint_loop_count = LaunchConfiguration("waypoint_loop_count")
    waypoint_loop_pause_sec = LaunchConfiguration("waypoint_loop_pause_sec")
    waypoint_loop_path_stable_sec = LaunchConfiguration("waypoint_loop_path_stable_sec")
    waypoint_loop_min_points = LaunchConfiguration("waypoint_loop_min_points")

    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "robot_description": robot_description,
            }
        ],
    )

    # RViz-only mode (default): no real sensors, no Gazebo, fake odom + map server + navigation.
    map_to_odom_static_tf = Node(
        condition=UnlessCondition(use_hardware),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    fake_base_odom = Node(
        condition=UnlessCondition(use_hardware),
        package="nav_simplified_model",
        executable="fake_base_odom.py",
        name="fake_base_odom",
        output="screen",
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    map_server = Node(
        condition=UnlessCondition(use_hardware),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "yaml_filename": map_file,
            }
        ],
    )

    map_server_lifecycle = Node(
        condition=UnlessCondition(use_hardware),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share_dir, "launch", "navigation_launch.py")
        ),
        condition=UnlessCondition(use_hardware),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "True",
            "params_file": params_file,
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    # Hardware mode: Livox + FAST_LIO + pointcloud2laserscan + Nav2 bringup.
    livox_driver_node = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_hardware,
                    "'.lower() == 'true' and '",
                    start_livox_driver,
                    "'.lower() == 'true'",
                ]
            )
        ),
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": ParameterValue(livox_publish_freq, value_type=float),
                "output_data_type": 0,
                "frame_id": livox_frame_id,
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": os.path.join(livox_share_dir, "config", "MID360_config.json"),
                "cmdline_input_bd_code": "livox0000000001",
            }
        ],
    )

    # Start FAST_LIO node directly to avoid any nested launch file forcing RViz on.
    fastlio_node = Node(
        condition=IfCondition(use_hardware),
        package="fast_lio",
        executable="fastlio_mapping",
        output="screen",
        parameters=[
            PathJoinSubstitution([fast_lio_share_dir, "config", fastlio_config]),
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    odom_to_camera_init_tf = Node(
        condition=IfCondition(use_hardware),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_camera_init_tf",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
        output="screen",
    )

    # Compensate sensor mounting pitch for local obstacle extraction only.
    body_to_level_tf = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_hardware,
                    "'.lower() == 'true' and '",
                    local_scan_target_frame,
                    "' == 'body_level'",
                ]
            )
        ),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_level_tf",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            lidar_pitch_compensation,
            "0",
            "body",
            "body_level",
        ],
        output="screen",
    )

    scan_node = Node(
        condition=IfCondition(use_hardware),
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", "/cloud_registered_body"),
            # Keep the raw scan on an internal topic so we can restamp it before Nav2 reads it.
            ("scan", "/scan_raw"),
        ],
        parameters=[
            {
                # Reduce TF-timing drops under real hardware jitter.
                "transform_tolerance": 0.3,
                # Use FAST_LIO body frame directly to avoid base_link tree disconnects.
                "target_frame": "body",
                # Keep a practical vertical band for near obstacle extraction.
                "min_height": -0.4,
                "max_height": 1.2,
                # Keep latency low: avoid old queued clouds becoming stale scans.
                "queue_size": 1,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.00436,
                "scan_time": 0.05,
                "range_min": 0.1,
                "range_max": 50.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
                "output_reliability": "reliable",
            }
        ],
    )

    # Dedicated local scan with stricter vertical filtering to suppress ground noise.
    scan_local_node = Node(
        condition=IfCondition(use_hardware),
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_local",
        output="screen",
        remappings=[
            ("cloud_in", "/cloud_registered_body"),
            # Keep both outputs raw for timestamp relaying.
            ("scan", "/scan_local_raw"),
            ("filtered_cloud", "/scan_local_cloud_raw"),
        ],
        parameters=[
            {
                "transform_tolerance": 0.2,
                # Default to FAST_LIO body frame; body_level is now opt-in only.
                "target_frame": local_scan_target_frame,
                # Keep broad vertical band; terrain quantile filter will classify obstacle by relative height.
                "min_height": local_scan_min_height,
                "max_height": local_scan_max_height,
                "adaptive_ground_filter": False,
                "terrain_ground_filter": True,
                "terrain_planar_size": local_scan_planar_size,
                "terrain_planar_width": 51,
                "terrain_quantile_z": local_scan_ground_percentile,
                "terrain_use_sorting": True,
                "terrain_limit_ground_lift": True,
                "terrain_max_ground_lift": local_scan_ground_margin,
                "terrain_min_rel_z": local_scan_terrain_min_rel_z,
                "terrain_max_rel_z": local_scan_terrain_max_rel_z,
                "terrain_dis_ratio_z": local_scan_terrain_dis_ratio_z,
                "terrain_vehicle_height": local_scan_vehicle_height,
                "terrain_min_ground_points": local_scan_min_ground_points,
                "terrain_min_block_points": local_scan_min_block_points,
                "terrain_min_obstacle_points": local_scan_min_obstacle_points,
                "terrain_neighbor_spread": local_scan_neighbor_spread,
                "terrain_min_obstacle_rel_z": local_scan_min_obstacle_rel_z,
                "terrain_obstacle_range_slope": local_scan_obstacle_range_slope,
                "terrain_consider_drop": False,
                "publish_filtered_cloud": True,
                "queue_size": 1,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.00654,
                "scan_time": 0.05,
                "range_min": local_scan_range_min,
                "range_max": local_scan_range_max,
                "use_inf": True,
                "inf_epsilon": 1.0,
                "output_reliability": "best_effort",
            }
        ],
    )

    scan_stamp_relay_node = ExecuteProcess(
        condition=IfCondition(use_hardware),
        cmd=[
            "python3",
            stamp_relay_script,
            "--ros-args",
            "-r",
            "__node:=scan_stamp_relay",
            "-p",
            "input_type:=laser_scan",
            "-p",
            "input_topic:=/scan_raw",
            "-p",
            "output_topic:=/scan",
            "-p",
            "queue_size:=1",
        ],
        output="screen",
    )

    # Re-stamp the local obstacle cloud too. Nav2 costmaps are sensitive to stale TF times.
    scan_local_stamp_relay_node = ExecuteProcess(
        condition=IfCondition(use_hardware),
        cmd=[
            "python3",
            stamp_relay_script,
            "--ros-args",
            "-r",
            "__node:=scan_local_stamp_relay",
            "-p",
            "input_type:=laser_scan",
            "-p",
            "input_topic:=/scan_local_raw",
            "-p",
            "output_topic:=/scan_local",
            "-p",
            "queue_size:=1",
        ],
        output="screen",
    )

    # The filtered point cloud is also used by the obstacle layer, so it needs the same treatment.
    scan_local_cloud_stamp_relay_node = ExecuteProcess(
        condition=IfCondition(use_hardware),
        cmd=[
            "python3",
            stamp_relay_script,
            "--ros-args",
            "-r",
            "__node:=scan_local_cloud_stamp_relay",
            "-p",
            "input_type:=point_cloud2",
            "-p",
            "input_topic:=/scan_local_cloud_raw",
            "-p",
            "output_topic:=/scan_local_cloud",
            "-p",
            "queue_size:=1",
        ],
        output="screen",
    )

    hardware_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share_dir, "launch", "bringup_launch.py")
        ),
        condition=IfCondition(use_hardware),
        launch_arguments={
            "map": map_file,
            "params_file": hardware_params_file,
            "use_sim_time": use_sim_time,
            "autostart": "True",
            "cmd_vel_topic": nav2_cmd_vel_topic,
        }.items(),
    )

    hardware_nav2_launch_delayed = TimerAction(
        condition=IfCondition(use_hardware),
        period=hardware_nav2_start_delay_sec,
        actions=[hardware_nav2_launch],
    )

    cmd_vel_bridge_node = Node(
        condition=IfCondition(use_hardware),
        package="nav_simplified_model",
        executable="twist_to_twist_stamped.py",
        name="cmd_vel_to_ap_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "input_topic": nav2_cmd_vel_topic,
                "output_topic": ap_cmd_vel_topic,
                "frame_id": "base_link",
            }
        ],
    )

    waypoint_loop_node = Node(
        condition=IfCondition(enable_waypoint_loop),
        package="nav_simplified_model",
        executable="waypoint_loop_runner.py",
        name="waypoint_loop_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "action_name": "/follow_waypoints",
                # RViz Nav2 panel publishes accumulated path on /waypoints.
                "waypoints_topic": waypoint_loop_topic,
                "waypoints_topic_type": waypoint_loop_topic_type,
                # Optional static list: \"x1,y1,yaw1,x2,y2,yaw2,...\"
                "waypoints_xyyaw": waypoint_loop_waypoints,
                "loop_forever": ParameterValue(waypoint_loop_forever, value_type=bool),
                "loop_count": ParameterValue(waypoint_loop_count, value_type=int),
                "pause_sec": ParameterValue(waypoint_loop_pause_sec, value_type=float),
                "path_stable_sec": ParameterValue(waypoint_loop_path_stable_sec, value_type=float),
                "min_waypoints": ParameterValue(waypoint_loop_min_points, value_type=int),
            }
        ],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_namespace": "false",
            "rviz_config": rviz_config,
        }.items(),
    )

    # RViz also needs to wait for FAST_LIO/TF to stabilize, otherwise it will cache
    # early messages and keep dropping them with stale-transform warnings.
    rviz_launch_delayed = TimerAction(
        condition=IfCondition(use_hardware),
        period=hardware_nav2_start_delay_sec,
        actions=[rviz_launch],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Occupancy map yaml generated from your PGM map.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_rviz,
                description="Nav2 parameters for RViz-only mode (no real hardware).",
            ),
            DeclareLaunchArgument(
                "hardware_params_file",
                default_value=default_params_hardware,
                description="Nav2 parameters used only in hardware mode.",
            ),
            DeclareLaunchArgument(
                "urdf",
                default_value=default_urdf,
                description="Robot URDF file to publish in RViz.",
            ),
            DeclareLaunchArgument(
                "fastlio_config",
                default_value="mid360.yaml",
                description="FAST_LIO config file under fast_lio/config.",
            ),
            DeclareLaunchArgument(
                "use_hardware",
                default_value="false",
                description="false: RViz-only mode, true: Livox+FAST_LIO hardware mode.",
            ),
            DeclareLaunchArgument(
                "start_livox_driver",
                default_value="true",
                description="In hardware mode, start livox_ros_driver2 or not.",
            ),
            DeclareLaunchArgument(
                "livox_publish_freq",
                default_value="20.0",
                description="Livox point cloud publish frequency (Hz), e.g. 10/20/50.",
            ),
            DeclareLaunchArgument(
                "livox_frame_id",
                default_value="livox_frame",
                description="Frame id for Livox messages.",
            ),
            DeclareLaunchArgument(
                "hardware_nav2_start_delay_sec",
                default_value="12.0",
                description="Delay hardware Nav2 bringup so FAST_LIO/TF can stabilize first.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz in this launch. Set false when RViz runs on another PC.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(nav2_share_dir, "rviz", "nav2_default_view.rviz"),
                description="RViz config file. Use src/FAST_LIO/rviz/fastlio.rviz for pointcloud view.",
            ),
            DeclareLaunchArgument(
                "ap_cmd_vel_topic",
                default_value="/ap/cmd_vel",
                description="Final real-robot velocity topic (TwistStamped).",
            ),
            DeclareLaunchArgument(
                "nav2_cmd_vel_topic",
                default_value="/cmd_vel_ap_raw",
                description="Intermediate Nav2 cmd_vel topic (Twist).",
            ),
            DeclareLaunchArgument(
                "enable_waypoint_loop",
                default_value="false",
                description="Enable automatic looping waypoint runner.",
            ),
            DeclareLaunchArgument(
                "waypoint_loop_topic",
                default_value="/waypoints",
                description="Waypoints Path topic (published by RViz Nav2 panel).",
            ),
            DeclareLaunchArgument(
                "waypoint_loop_topic_type",
                default_value="marker_array",
                description="Waypoint topic type: marker_array (RViz panel), path, or pose_stamped (/goal_pose).",
            ),
            DeclareLaunchArgument(
                "waypoint_loop_waypoints",
                default_value="",
                description="Optional static waypoints string: x1,y1,yaw1,x2,y2,yaw2,...",
            ),
            DeclareLaunchArgument(
                "waypoint_loop_forever",
                default_value="true",
                description="Loop waypoints forever when true.",
            ),
            DeclareLaunchArgument(
                "waypoint_loop_count",
                default_value="0",
                description="Loop count when not forever (0 means disabled unless forever=true).",
            ),
            DeclareLaunchArgument(
                "waypoint_loop_pause_sec",
                default_value="0.5",
                description="Pause between loop rounds (seconds).",
            ),
            DeclareLaunchArgument(
                "waypoint_loop_path_stable_sec",
                default_value="1.5",
                description="Required quiet time before consuming RViz path (seconds).",
            ),
            DeclareLaunchArgument(
                "waypoint_loop_min_points",
                default_value="2",
                description="Minimum waypoint count before looping.",
            ),
            DeclareLaunchArgument(
                "lidar_pitch_compensation",
                default_value="0.0",
                description="Optional pitch compensation (rad) from body to body_level for local scan.",
            ),
            DeclareLaunchArgument(
                "local_scan_target_frame",
                default_value="body",
                description="Target frame for local obstacle extraction. Use body_level only if extra leveling is required.",
            ),
            DeclareLaunchArgument(
                "local_scan_min_height",
                default_value="-0.4",
                description="Local scan vertical filter min height (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_max_height",
                default_value="0.8",
                description="Local scan vertical filter max height (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_ground_percentile",
                default_value="0.40",
                description="Ground quantile (0~1) for terrain filter in /scan_local.",
            ),
            DeclareLaunchArgument(
                "local_scan_ground_margin",
                default_value="0.36",
                description="Max ground lift above min-z for terrain filter (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_min_obstacle_rel_z",
                default_value="0.08",
                description="Min relative height above local ground to mark obstacle (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_min_block_points",
                default_value="3",
                description="Min points in a planar cell before obstacle decision.",
            ),
            DeclareLaunchArgument(
                "local_scan_min_obstacle_points",
                default_value="1",
                description="Min obstacle points in a planar cell before publishing obstacle.",
            ),
            DeclareLaunchArgument(
                "local_scan_obstacle_range_slope",
                default_value="0.01",
                description="Extra obstacle height threshold added per meter of range to suppress tilted-floor noise.",
            ),
            DeclareLaunchArgument(
                "local_scan_terrain_max_rel_z",
                default_value="0.30",
                description="Max relative z used in local ground estimation (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_terrain_dis_ratio_z",
                default_value="0.20",
                description="Range-relaxed z window ratio for sloped ground.",
            ),
            DeclareLaunchArgument(
                "local_scan_planar_size",
                default_value="0.20",
                description="Planar cell size for local terrain filter (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_range_min",
                default_value="0.2",
                description="Ignore local obstacle points within this range (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_range_max",
                default_value="4.5",
                description="Max range used for local obstacle extraction (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_terrain_min_rel_z",
                default_value="-0.8",
                description="Min relative z used in local ground estimation (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_min_ground_points",
                default_value="4",
                description="Min points to estimate local ground in each planar cell.",
            ),
            DeclareLaunchArgument(
                "local_scan_vehicle_height",
                default_value="1.0",
                description="Max obstacle height above local ground for local filtering (m).",
            ),
            DeclareLaunchArgument(
                "local_scan_neighbor_spread",
                default_value="false",
                description="Whether to spread ground samples to neighboring planar cells.",
            ),
            DeclareLaunchArgument(
                "local_scan_ground_min_range",
                default_value="0.35",
                description="Reserved for compatibility.",
            ),
            DeclareLaunchArgument(
                "local_scan_ground_max_range",
                default_value="2.8",
                description="Reserved for compatibility.",
            ),
            robot_state_publisher_node,
            map_to_odom_static_tf,
            fake_base_odom,
            map_server,
            map_server_lifecycle,
            navigation_launch,
            livox_driver_node,
            fastlio_node,
            odom_to_camera_init_tf,
            body_to_level_tf,
            scan_node,
            scan_local_node,
            scan_stamp_relay_node,
            scan_local_stamp_relay_node,
            scan_local_cloud_stamp_relay_node,
            hardware_nav2_launch_delayed,
            cmd_vel_bridge_node,
            waypoint_loop_node,
            rviz_launch_delayed,
        ]
    )
