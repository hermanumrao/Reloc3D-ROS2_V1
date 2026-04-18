from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

config = os.path.join(
    get_package_share_directory("relocalization_3d"), "config", "params.yaml"
)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="relocalization_3d",
                executable="relocalization_node",
                name="relocalization_node",
                output="screen",
                parameters=["config/params.yaml"],
            )
        ]
    )
