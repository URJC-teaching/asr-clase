# Copyright 2025 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('camera')
    # param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    optical_frame_arg = DeclareLaunchArgument(
        'optical_frame',
        default_value='camera_rgb_optical_frame',
        description='Camera optical frame (default: camera_rgb_optical_frame)'
    )

    kobuki_sim_arg = DeclareLaunchArgument(
        'kobuki_sim',
        default_value='false',
        description='If true, override frame_id with optical_frame in yolo_to_standard. (default: false)'
    )

    yolo_cmd = Node(package='camera',
        executable='yolo_to_standard_node_3d',
        output='screen',
        parameters=[{
        'kobuki_sim': LaunchConfiguration('kobuki_sim'),
        'optical_frame': LaunchConfiguration('optical_frame')
        }],
        remappings=[
          ('input_detection', '/yolo/detections_3d'),
          ('output_detection_3d', '/detections_3d')
        ])

    ld = LaunchDescription()
    ld.add_action(optical_frame_arg)
    ld.add_action(kobuki_sim_arg)
    ld.add_action(yolo_cmd)

    return ld
