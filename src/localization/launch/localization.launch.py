import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("localization")
    config = os.path.join(pkg, "config", "amcl.yaml")
    map_file = os.path.join(pkg, "maps", "my_map.yaml")

    lifecycle_nodes = ["map_server", "amcl"]

    return LaunchDescription([
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[config, {"yaml_filename": map_file}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": lifecycle_nodes,
                "bond_timeout": 10.0,
                "attempt_respawn_reconnection": True,
                "bond_respawn_max_duration": 30.0,
            }],
        ),
    ])
