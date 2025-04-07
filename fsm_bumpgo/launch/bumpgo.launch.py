
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bumpgo_cmd = Node(
            package='fsm_bumpgo',
            executable='bump_go_node',
            name='bump_go_node',
            output='screen',
            remappings=[
                ('/bumper', '/events/bumper'), 
                ('/out_vel', '/cmd_vel'),
            ]
        )

    ld = LaunchDescription()
    ld.add_action(bumpgo_cmd)

    return ld
