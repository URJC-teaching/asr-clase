# Copyright 2025 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node

from yolo_msgs.msg import DetectionArray
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        self.detection_sub = self.create_subscription(
            DetectionArray,
            'input_detection',
            self.detection_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            'output_detection_2d',
            rclpy.qos.qos_profile_sensor_data
        )

    def detection_callback(self, msg: DetectionArray):
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        for detection in msg.detections:
            detection_msg = Detection2D()
            detection_msg.header = msg.header

            detection_msg.bbox.center.position.x = detection.bbox.center.position.x
            detection_msg.bbox.center.position.y = detection.bbox.center.position.y
            detection_msg.bbox.size_x = detection.bbox.size.x
            detection_msg.bbox.size_y = detection.bbox.size.y

            obj_msg = ObjectHypothesisWithPose()
            obj_msg.hypothesis.class_id = detection.class_name
            obj_msg.hypothesis.score = detection.score

            detection_msg.results.append(obj_msg)
            detection_array_msg.detections.append(detection_msg)

        self.detection_pub.publish(detection_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
