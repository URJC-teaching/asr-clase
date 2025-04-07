
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    publisher_cmd = Node(
            package='node_programming',
            executable='lifecycle_publisher_node',
            name='lifecycle_publisher_node',
            output='screen',
        )

    subscriber_cmd = Node(
            package='node_programming',
            executable='subscriber_node',
            name='subscriber_node',
            output='screen',
        )

    ld = LaunchDescription()
    ld.add_action(publisher_cmd)
    ld.add_action(subscriber_cmd)

    return ld
