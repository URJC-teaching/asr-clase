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
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import math
from cv_bridge import CvBridge
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer


'''
In this example, instead of using 3D detections directly, we use 2D detections
from YOLO along with depth images to compute 3D positions.
'''

class AltThreeDYOLOClassDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_3d_class_detector_node')

        self.declare_parameter('target_class', 'person')
        self.declare_parameter('base_frame', 'base_footprint')

        self.target_class = self.get_parameter('target_class').value
        self.base_frame = self.get_parameter('base_frame').value

        self.f_x = None  # Focal length in x (fx)
        self.c_x = None  # Principal point x-coordinate (cx)
        self.f_y = None  # Focal length in y (fy)
        self.c_y = None  # Principal point y-coordinate (cy)
        self.bridge = CvBridge()
        
        self.get_logger().info("Waiting for CameraInfo and starting synchronization...")

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for the 3D position vector
        self.attractive_pub = self.create_publisher(Vector3, 'attractive_vector', 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera_info', 
            self.camera_info_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # We need to buffer these since they will be synchronized in a dedicated callback
        self.detection_sub = Subscriber(
            self, Detection2DArray, 'input_detection_2d', qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, Image, 'input_depth_image', qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        
        # Synchronizer setup: 
        #   - ts: The synchronizer object
        #   - self.synced_callback: The function that runs when all messages arrive
        #   - 10: Queue size
        #   - [self.detection_sub, self.depth_sub]: The messages to synchronize
        self.ts = ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub], 10, 0.1 # 0.1s maximum allowed difference
        )
        self.ts.registerCallback(self.synced_callback)


    def camera_info_callback(self, msg: CameraInfo):
        # The intrinsic matrix K is a 9-element array (row-major order)
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.f_x = msg.k[0] # fx
        self.c_x = msg.k[2] # cx
        self.f_y = msg.k[4] # fy
        self.c_y = msg.k[5] # cy
        
        self.get_logger().info(f'Got camera intrinsics: fx={self.f_x:.2f}, fy={self.f_y:.2f}')
        self.destroy_subscription(self.camera_info_sub) # Intrinsics are static

    
    def synced_callback(self, detection_msg: Detection2DArray, depth_msg: Image):
        # 0. Check for parameters
        if self.f_x is None or self.c_x is None or self.f_y is None or self.c_y is None:
            self.get_logger().warn('Camera intrinsics not received yet. Skipping processing.')
            return

        if not detection_msg.detections:
            return

        # 1. Find the target detection
        target_detection = None
        for detection in detection_msg.detections:
            if detection.results and detection.results[0].hypothesis.class_id == self.target_class:
                target_detection = detection
                break
        
        if target_detection is None:
            return

        # 2. Extract 2D Pixel Coordinates
        x_pixel = int(target_detection.bbox.center.position.x)
        y_pixel = int(target_detection.bbox.center.position.y)
        
        if x_pixel >= depth_msg.width or y_pixel >= depth_msg.height:
             self.get_logger().error(f"Pixel ({x_pixel}, {y_pixel}) is outside depth map bounds.")
             return

        # 3. Convert ROS Depth Image to OpenCV (NumPy array)
        try:
            # Assumes 16UC1 (unsigned 16-bit integer, common for mm depth) or 32FC1 (float meter depth)
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return

        # 4. Extract Depth (Z)
        # Check if depth image is valid before reading
        if cv_depth_image is None or cv_depth_image.size == 0:
            self.get_logger().error("Converted depth image is empty.")
            return

        # Get the raw value at the center pixel
        raw_z = cv_depth_image[y_pixel, x_pixel]

        # Determine scale factor based on encoding type
        if cv_depth_image.dtype == np.uint16:
            # Common for raw depth in mm. Convert to meters (1000 mm = 1 m)
            Z = float(raw_z) / 1000.0
        elif cv_depth_image.dtype == np.float32 or cv_depth_image.dtype == np.float64:
            # Common for rectified depth in meters
            Z = float(raw_z)
        else:
             self.get_logger().warn(f"Unknown depth image encoding ({cv_depth_image.dtype}). Assuming data is in meters.")
             Z = float(raw_z)

        # Check for invalid depth readings (0 or NaN)
        if Z <= 0.0 or np.isnan(Z):
            self.get_logger().warn(f"Invalid depth reading (Z={Z:.2f}m) at pixel ({x_pixel}, {y_pixel}). Skipping.")
            return

        # 5. Project to 3D Coordinates (X, Y, Z)
        
        # Calculate X and Y using the Pinhole Camera Model
        # X = (u - cx) * Z / fx
        X = (x_pixel - self.c_x) * Z / self.f_x
        
        # Y = (v - cy) * Z / fy
        Y = (y_pixel - self.c_y) * Z / self.f_y

        # 6. Transform to desired frame
        target_point = PointStamped()
        target_point.point.x = X
        target_point.point.y = Y
        target_point.point.z = Z # Z (depth) is the distance along the camera's optical axis

        source_frame = detection.header.frame_id
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
        
        vec = Vector3()
        vec.x = transformed_point.point.x
        vec.y = transformed_point.point.y
        vec.z = transformed_point.point.z

        self.get_logger().debug(f'Attractive vector for {self.target_class} '
                                   f'x={vec.x:.2f}, y={vec.y:.2f}, z={vec.z:.2f}')
        
        dist = math.sqrt(vec.x**2 + vec.y**2 + vec.z**2)
        self.get_logger().info(
            f'Instance of class "{self.target_class}" detected at {dist:.2f} m)'
        )

        self.attractive_pub.publish(vec)

        


def main(args=None):
    rclpy.init(args=args)
    node = AltThreeDYOLOClassDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()