# Copyright 2025 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node

from yolo_msgs.msg import DetectionArray
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose

class YoloDetectionNode3D(Node):

    def __init__(self):
        super().__init__('yolo_detection_node_3d')

        self.detection_sub = self.create_subscription(
            DetectionArray,
            'input_detection',
            self.detection_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.detection_pub = self.create_publisher(
            Detection3DArray,
            'output_detection_3d',
            rclpy.qos.qos_profile_sensor_data
        )

    def detection_callback(self, msg: DetectionArray):
        detection_array_msg = Detection3DArray()
        detection_array_msg.header = msg.header

        for detection in msg.detections:
            detection_msg = Detection3D()
            detection_msg.header = msg.header
            detection_msg.header.frame_id = detection.bbox3d.frame_id

            detection_msg.bbox.center.position.x = detection.bbox3d.center.position.x
            detection_msg.bbox.center.position.y = detection.bbox3d.center.position.y
            detection_msg.bbox.center.position.z = detection.bbox3d.center.position.z

            detection_msg.bbox.size.x = detection.bbox3d.size.x
            detection_msg.bbox.size.y = detection.bbox3d.size.y
            detection_msg.bbox.size.z = detection.bbox3d.size.z

            self.get_logger().debug(f'Detected {detection.class_name} at '
                                   f'x={detection.bbox3d.center.position.x:.2f}, '
                                   f'y={detection.bbox3d.center.position.y:.2f}, '
                                   f'z={detection.bbox3d.center.position.z:.2f} '
                                   f'({detection.bbox3d.frame_id})')

            obj_msg = ObjectHypothesisWithPose()
            obj_msg.hypothesis.class_id = detection.class_name
            obj_msg.hypothesis.score = detection.score

            obj_msg.pose.pose.position.x = detection.bbox3d.center.position.x
            obj_msg.pose.pose.position.y = detection.bbox3d.center.position.y
            obj_msg.pose.pose.position.z = detection.bbox3d.center.position.z

            detection_msg.results.append(obj_msg)
            detection_array_msg.detections.append(detection_msg)

        self.detection_pub.publish(detection_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
