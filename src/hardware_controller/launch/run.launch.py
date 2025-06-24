from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    irsensor = Node(
        package='hardware_controller',
        executable='irsensor',
        output='screen',
    )
    motor_controller = Node(
        package='hardware_controller',
        executable='motors_controller',
        output='screen',
    )
    bumpgo = Node(
        package='fsm_bumpgo_cpp',
        executable='bumpgo',
        output='screen',
        remappings=[
            ('/input_scan', '/ir_scan'),
            ('/output_vel', '/cmd_vel'),
        ]
    )
    ld.add_action(irsensor)
    ld.add_action(motor_controller)
    ld.add_action(bumpgo)
    return ld
