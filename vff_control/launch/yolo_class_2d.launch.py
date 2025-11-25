from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
   
    return LaunchDescription([
  
        # YOLO class detector node (publishes attractive vectors). Needs YOLO to be running
        Node(
            package='vff_control',
            executable='yolo_class_detector_node_2d',
            name='yolo_class_detector_node_2d',
            output='screen',
            parameters=[{
                'target_class': 'cup',
                'base_frame': 'base_footprint',
                'optical_frame': 'camera_rgb_optical_frame'
            }],
            remappings=[
            ('/input_detection_2d', '/detections_2d'),
            ('/input_image', '/rgbd_camera/image'),
            ('/camera_info', '/rgbd_camera/camera_info'),
            ]
        ),
    ])