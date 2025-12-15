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
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Vector3, PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import Image, CameraInfo
import math


class TwoDYOLOClassDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_class_detector_node')

        self.declare_parameter('target_class', 'person')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('optical_frame', 'camera_rgb_optical_frame')
        
        self.target_class = self.get_parameter('target_class').value
        self.base_frame = self.get_parameter('base_frame').value
        self.optical_frame = self.get_parameter('optical_frame').value

        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.configured = False

        # Subscriber to the image
        # self.image_sub = self.create_subscription(
        #     Image,
        #     'input_image',
        #     self.image_callback,
        #     rclpy.qos.qos_profile_sensor_data
        # )

        # Subscriber to the camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_info_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Subscriber to Detection2DArray
        self.sub = self.create_subscription(
            Detection2DArray,
            'input_detection_2d',
            self.detection_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Publisher for attractive vector
        self.attractive_pub = self.create_publisher(Vector3, 'attractive_vector', 10)

        self.get_logger().info(f'2D YOLO Class Detector Node initialized, looking for class: {self.target_class}')

    # def image_callback(self, msg: Image):
    #     # Just to get image size for angle calculation
    #     self.current_image = msg
    #     self.current_image_size = (msg.width, msg.height)
    #     self.get_logger().info(f'Got image of size: {msg.width}x{msg.height}')
    #     # Unsubscribe after first callback
    #     self.destroy_subscription(self.image_sub)

    def camera_info_callback(self, msg: CameraInfo):
        # The intrinsic matrix K is a 9-element array (row-major order)
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.f_x = msg.k[0] # fx is K[0]
        self.c_x = msg.k[2] # cx is K[2]

        self.current_image_size = (msg.width, msg.height)
        self.get_logger().info(f'Got image of size: {msg.width}x{msg.height}')
        self.get_logger().info(f'Got camera intrinsics: fx={self.f_x:.2f}, cx={self.c_x:.2f}')
        self.configured = True
        self.destroy_subscription(self.camera_info_sub)

    def detection_callback(self, msg: Detection2DArray):
        if not msg.detections:
            return
        
        self.get_logger().debug(f'Received {len(msg.detections)} detections')

        # Find first detection of the target class
        for detection in msg.detections:
            if detection.results and detection.results[0].hypothesis.class_id == self.target_class:
                self.publish_attractive_vector(detection)
                break

    def publish_attractive_vector(self, detection):
        
        if not self.configured:
            self.get_logger().warn('Camera info not yet received, cannot compute angles')
            return
        # Calculate angle relative to image center (positive left, negative right)
        x_pixel = detection.bbox.center.position.x
        self.get_logger().debug(f'Detection center x: {x_pixel:.2f}. BB width: {detection.bbox.size_x:.2f}')
        

        # Use camera intrinsics to compute angle
        f_x = self.f_x
        c_x = self.c_x

        pixel_offset_x = x_pixel - c_x
        angle = math.atan(pixel_offset_x / f_x)
        self.get_logger().debug(f'Detected {self.target_class} at angle {math.degrees(angle):.1f} degrees ({self.optical_frame})')

        # Compute attractive vector at fixed distance of 1 meter (relative to camera optical frame)
        vec = Vector3()
        vec.x = math.tan(angle)
        vec.y = 0.0  
        vec.z = 1.0 # Fixed distance of 1 meter. No depth info from 2D detection
        
        # Alternative simpler angle calculation without intrinsics (but less accurate)
        # center_x = self.current_image_size[0] / 2.0 if hasattr(self, 'current_image_size') else 320.0
        # angle = (center_x - x_pixel) / center_x  # +1 left edge, 0 center, -1 right edge
        # angle = angle * 90.0  # Convert to degrees: +90 left, 0 center, -90 right (assuming 90° FOV)
        # self.get_logger().info(f'Detected {self.target_class} at angle {angle:.1f} degrees')

        # vec = Vector3()
        # vec.x = math.tan(angle)
        # vec.y = 0.0  
        # vec.z = 1.0 # Fixed distance of 1 meter. No depth info from 2D detection

        target_point = PointStamped()
        target_point.header = detection.header
        target_point.point.x = vec.x
        target_point.point.y = vec.y
        target_point.point.z = vec.z

        # source_frame = detection.header.frame_id
        source_frame = self.optical_frame
        target_frame = self.base_frame
        detection_time = detection.header.stamp

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
        
        vec.x = transformed_point.point.x
        vec.y = transformed_point.point.y
        vec.z = transformed_point.point.z

        angle_base = math.atan2(vec.y, vec.x)

        self.get_logger().info(f'Detected {self.target_class} at angle {math.degrees(angle_base):.1f} degrees ({target_frame})')
        
        self.get_logger().debug(f'Attractive vector for {self.target_class} '
                                   f'x={vec.x:.2f}, y={vec.y:.2f}, z={vec.z:.2f} ({target_frame})')

        self.attractive_pub.publish(vec)

        


def main(args=None):
    rclpy.init(args=args)
    node = TwoDYOLOClassDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
