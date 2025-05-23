# Copyright 2025 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.declare_parameter('min_distance', 0.5)
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value

        self.get_logger().info(f'obstacle_detector_node set to {self.min_distance:.2f} m')

        self.laser_sub = self.create_subscription(
            LaserScan,
            'input_scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.obstacle_pub = self.create_publisher(Bool, 'obstacle', 10)

    def laser_callback(self, scan: LaserScan):
        if not scan.ranges:
            return

        distance_min = min(scan.ranges)
        min_idx = scan.ranges.index(distance_min)

        msg = Bool()
        if distance_min < self.min_distance:
            angle = scan.angle_min + scan.angle_increment * min_idx

            # Normalize angle
            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi

            self.get_logger().info(f'Obstacle in ({distance_min:.2f}, {angle:.2f})')
            msg.data = True
        else:
            msg.data = False

        self.obstacle_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
