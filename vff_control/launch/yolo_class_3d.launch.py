from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
   
    return LaunchDescription([
  
        Node(
            package='vff_control',
            executable='yolo_class_detector_node_3d',
            name='yolo_class_detector_node_3d',
            output='screen',
            parameters=[{
                'target_class': 'cup',
                'base_frame': 'base_footprint'
            }],
            remappings=[
            ('/input_detection_3d', '/detections_3d'),
            ]
        )
    ])
