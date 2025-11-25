from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
   
    return LaunchDescription([
  
        Node(
            package='vff_control',
            executable='yolo_class_detector_node_3d_alt',
            name='yolo_class_detector_node_3d_alt',
            output='screen',
            parameters=[{
                'target_class': 'chair',
                'base_frame': 'base_footprint'
            }],
            remappings=[
            ('/input_detection_2d', '/detections_2d'),
            ('/input_depth_image', '/rgbd_camera/depth_image'),
            ('/camera_info', '/rgbd_camera/camera_info')
            ]
        )
    ])
