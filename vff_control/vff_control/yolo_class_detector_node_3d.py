# Copyright 2025 Rodrigo Pérez-Rodríguez
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

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Vector3, PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import Image
import math


class TwoDYOLOClassDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_class_detector_node')

        # Parameter: target YOLO class
        self.declare_parameter('target_class', 'person')
        self.target_class = self.get_parameter('target_class').value

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber to Detection3DArray
        self.sub = self.create_subscription(
            Detection3DArray,
            'input_detection_3d',
            self.detection_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Publisher for attractive vector
        self.attractive_pub = self.create_publisher(Vector3, 'attractive_vector', 10)

    def detection_callback(self, msg: Detection3DArray):
        if not msg.detections:
            return

        # Find first detection of the target class
        for detection in msg.detections:
            if detection.results and detection.results[0].hypothesis.class_id == self.target_class:
                self.publish_attractive_vector(detection)
                break

    def publish_attractive_vector(self, detection):
        
        # Get the target coordinates in the source frame
        target_point = PointStamped()
        target_point.header = detection.header
        target_point.point = detection.bbox.center

        source_frame = detection.header.frame_id
        target_frame = 'base_link'  # Robot's base frame

        try:
            # Lookup the transform
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            # Transform the point to the target frame
            transformed_point = do_transform_point(target_point, transform)
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}')
            return
       
        vec = Vector3()
        vec.x = transformed_point.point.x
        vec.y = transformed_point.point.y
        vec.z = transformed_point.point.z

        self.get_logger().debug(f'Attractive vector for {self.target_class} '
                                   f'x={vec.x:.2f}, y={vec.y:.2f}, z={vec.z:.2f}')

        self.attractive_pub.publish(vec)

        


def main(args=None):
    rclpy.init(args=args)
    node = TwoDYOLOClassDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
