from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch argument
    erratic_arg = DeclareLaunchArgument( # Command line argument to toggle between normal and erratic PID constants
        'erratic',
        default_value='False',
        description='Set to True to use erratic PID constants that cause oscillation'
    )

    # Node for publishing TF
    publisher_cmd = Node(
        package='tf_seeker',
        executable='tf_publisher_node',
        name='tf_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'tf_update_time': 60.0}],
    )

    # Node for seeking TF
    seeker_cmd = Node(  
        package='tf_seeker',
        executable='tf_seeker_node',
        name='tf_seeker_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'erratic': LaunchConfiguration('erratic')
        }],
    )

    ld = LaunchDescription()
    ld.add_action(erratic_arg)
    ld.add_action(publisher_cmd)
    ld.add_action(seeker_cmd)

    return ld
