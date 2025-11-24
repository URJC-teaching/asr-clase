from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt_bumpgo',
            executable='bumpgo_groot',
            name='bt_bumpgo',
            output='screen',
            remappings=[
                ('/out_vel', '/cmd_vel'),
                ('/bumper', '/events/bumper')
            ]
        )
    ])
