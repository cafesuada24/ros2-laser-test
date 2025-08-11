import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "tf2_detector"


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "config", "detector.params.yaml"
    )
    detector_cmd = Node(
        package=PACKAGE_NAME,
        executable="detector",
        output="screen",
        parameters=[config_file],
    )

    ld = LaunchDescription()
    ld.add_action(detector_cmd)

    return ld
