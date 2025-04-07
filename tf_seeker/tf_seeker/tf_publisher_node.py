# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node

import random

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from builtin_interfaces.msg import Time


class TFPublisherNode(Node):

    def __init__(self):
        super().__init__('tf_producer')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.transform = TransformStamped()
        self.generate_tf()

        self.create_timer(20.0, self.generate_tf)
        self.create_timer(0.05, self.publish_tf)

    def generate_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = 'odom'
        self.transform.child_frame_id = 'target'

        self.transform.transform.translation.x = random.uniform(-5.0, 5.0)
        self.transform.transform.translation.y = random.uniform(-5.0, 5.0)
        self.transform.transform.translation.z = 0.0

        # No rotation (identity quaternion)
        self.transform.transform.rotation.x = 0.0
        self.transform.transform.rotation.y = 0.0
        self.transform.transform.rotation.z = 0.0
        self.transform.transform.rotation.w = 1.0

        self.get_logger().info(
            f"Generated transform to ({self.transform.transform.translation.x:.2f}, "
            f"{self.transform.transform.translation.y:.2f})")

    def publish_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.transform)


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
