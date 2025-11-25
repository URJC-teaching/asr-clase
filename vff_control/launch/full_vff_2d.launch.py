from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
   
    return LaunchDescription([
  
        # Obstacle detector node (publishes raw repulsive vectors)
        Node(
            package='vff_control',
            executable='obstacle_detector_node',
            name='obstacle_detector_node',
            output='screen',
            parameters=[{
            'min_distance': 0.5,
            'base_frame': 'base_footprint'
            }],
            remappings=[
            ('/input_laser', '/scan_raw')
            ]
        ),

        # YOLO launcher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_bringup'),
                'launch',
                'yolo.launch.py'
            )
            ),
            launch_arguments={
            'input_image_topic': '/rgbd_camera/image',
            'input_depth_topic': '/rgbd_camera/depth_image',
            'input_depth_info_topic': '/rgbd_camera/camera_info',
            'target_frame': 'camera_link'
            }.items()
        ),

        # YOLO messages to standard
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('camera'),
                    'launch',
                    'yolo_detection2d.launch.py'
                )
            )
        ),

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
            ('/camera_info', '/rgbd_camera/camera_info')
            ]
        ),

        # VFF controller node
        Node(
            package='vff_control',
            executable='vff_controller_node',
            name='vff_controller_node',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.1,
                'max_angular_speed': 1.0,
                'repulsive_gain_factor': 1.0,
                'repulsive_influence_distance': 0.2,
                'stay_distance': -1.0 # No stay distance in 2D
            }],
            remappings=[
            ('/vel', '/cmd_vel')
            ]
        ),
    ])