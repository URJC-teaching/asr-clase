from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('hri_examples')
    params_file = os.path.join(pkg_dir, 'config', 'hri.yaml')

    return LaunchDescription([
        Node(
            package='hri_examples',
            executable='generate_response_node',
            name='generate_response_node',
            output='screen',
            parameters=[params_file],
        )
    ])
