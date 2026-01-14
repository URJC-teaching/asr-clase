from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import os

def generate_launch_description():
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'waypoints.yaml'
    )
    return LaunchDescription([
        Node(
            package='fsm_nav',
            executable='fsm_nav_node',
            name='fsm_nav_node',
            output='screen',
            parameters=[config_path]
        )
    ])
