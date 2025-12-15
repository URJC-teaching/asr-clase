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


class ThreeDYOLOClassDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_class_detector_node')

        # Parameter: target YOLO class
        self.declare_parameter('target_class', 'person')
        self.target_class = self.get_parameter('target_class').value
        self.declare_parameter('base_frame', 'base_footprint')
        self.base_frame = self.get_parameter('base_frame').value

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
        target_point.point.x = detection.bbox.center.position.x
        target_point.point.y = detection.bbox.center.position.y
        target_point.point.z = detection.bbox.center.position.z

        source_frame = detection.header.frame_id
        target_frame = self.base_frame
        detection_time = detection.header.stamp

        dist = math.sqrt(target_point.point.x**2 + target_point.point.y**2 + target_point.point.z**2)
        angle = math.atan2(target_point.point.y, target_point.point.x)

        self.get_logger().debug(f'Original point for {self.target_class} '
                                   f'x={target_point.point.x:.2f}, y={target_point.point.y:.2f}, z={target_point.point.z:.2f} ({source_frame})')

        self.get_logger().info(f'Detected {self.target_class} at {dist:.2f} m, angle {math.degrees(angle):.1f} degrees ({source_frame})')
        try:
            # Lookup the transform
            self.get_logger().debug(f'Looking up transform from {source_frame} to {target_frame}')
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                detection_time,  # Use the actual timestamp from the sensor data
                timeout=rclpy.duration.Duration(seconds=0.5) 
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
        
        dist = math.sqrt(vec.x**2 + vec.y**2 + vec.z**2)
        angle_base = math.atan2(vec.y, vec.x)
        self.get_logger().debug(f'Detected {self.target_class} at {dist:.2f} m, angle {math.degrees(angle_base):.1f} degrees ({target_frame})')
        
        self.get_logger().debug(f'Attractive vector for {self.target_class} '
                                   f'x={vec.x:.2f}, y={vec.y:.2f}, z={vec.z:.2f} ({target_frame})')

        self.attractive_pub.publish(vec)

        


def main(args=None):
    rclpy.init(args=args)
    node = ThreeDYOLOClassDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
