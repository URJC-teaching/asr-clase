from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node for publishing TF
    publisher_cmd = Node(
        package='tf_seeker',
        executable='tf_publisher_node',
        name='tf_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Node for seeking TF
    seeker_cmd = Node(
        package='tf_seeker',
        executable='tf_seeker_node',
        name='tf_seeker_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ld = LaunchDescription()
    ld.add_action(publisher_cmd)
    ld.add_action(seeker_cmd)

    return ld
