import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = "vff_avoidance"

def generate_launch_description() -> LaunchDescription:
    config_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "config", "avoidance_vff.params.yaml"
    )
    vff_avoidance_cmd = Node(
        package='vff_avoidance',
        executable='avoidance_vff',
        parameters=[config_file],
        remappings=[
            ('input_scan_1', '/sonar_1'),
            ('input_scan_2', '/sonar_2'),
            ('input_scan_3', '/sonar_3'),
            ('output_vel', '/cmd_vel'),
        ],
    )
    ld = LaunchDescription()
    ld.add_action(vff_avoidance_cmd)

    return ld
