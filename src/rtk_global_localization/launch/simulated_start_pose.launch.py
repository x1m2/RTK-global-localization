import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    workspace_root = _guess_workspace_root(package_share_dir)

    default_map_yaml = os.path.join(workspace_root, "maps", "menkou.yaml")
    default_map_origin_path = os.path.join(package_share_dir, "config", "map_origin.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_map_server = LaunchConfiguration("start_map_server")
    map_yaml = LaunchConfiguration("map_yaml")
    map_origin_path = LaunchConfiguration("map_origin_path")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    map_x = LaunchConfiguration("map_x")
    map_y = LaunchConfiguration("map_y")
    map_z = LaunchConfiguration("map_z")
    yaw = LaunchConfiguration("yaw")
    rate = LaunchConfiguration("rate")
    publish_tf = LaunchConfiguration("publish_tf")

    map_server = Node(
        condition=IfCondition(start_map_server),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "yaml_filename": map_yaml,
                "topic_name": "map",
                "frame_id": map_frame,
            }
        ],
    )

    map_lifecycle_manager = Node(
        condition=IfCondition(start_map_server),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_simulated_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    simulated_start_pose = Node(
        package="rtk_global_localization",
        executable="simulated_start_pose.py",
        name="simulated_start_pose",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_origin_path": map_origin_path,
                "map_frame": map_frame,
                "base_frame": base_frame,
                "map_x": ParameterValue(map_x, value_type=float),
                "map_y": ParameterValue(map_y, value_type=float),
                "map_z": ParameterValue(map_z, value_type=float),
                "yaw": ParameterValue(yaw, value_type=float),
                "rate": ParameterValue(rate, value_type=float),
                "publish_tf": ParameterValue(publish_tf, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time. Keep false for standalone RViz demos.",
            ),
            DeclareLaunchArgument(
                "start_map_server",
                default_value="true",
                description="Publish the 2D occupancy grid map.",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=default_map_yaml,
                description="2D occupancy-grid map YAML.",
            ),
            DeclareLaunchArgument(
                "map_origin_path",
                default_value=default_map_origin_path,
                description="Geographic anchor used to convert map coordinates to WGS84.",
            ),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="sim_body"),
            DeclareLaunchArgument(
                "map_x",
                default_value="0.0",
                description="Simulated startup x position in map frame, meters.",
            ),
            DeclareLaunchArgument(
                "map_y",
                default_value="0.0",
                description="Simulated startup y position in map frame, meters.",
            ),
            DeclareLaunchArgument(
                "map_z",
                default_value="0.0",
                description="Simulated startup z position in map frame, meters.",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="0.0",
                description="Simulated startup yaw in map frame, radians.",
            ),
            DeclareLaunchArgument(
                "rate",
                default_value="2.0",
                description="Publish rate in Hz.",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Publish map -> sim_body TF.",
            ),
            map_server,
            map_lifecycle_manager,
            simulated_start_pose,
        ]
    )
