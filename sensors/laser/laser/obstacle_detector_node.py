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
        self.declare_parameter('real_robot', False)

        self.min_distance = self.get_parameter('min_distance').value
        self.real_robot = self.get_parameter('real_robot').value

        self.get_logger().info(f'Obstacle_detector_node set to {self.min_distance:.2f} m')

        self.laser_sub = self.create_subscription(
            LaserScan,
            'input_laser',
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
            if not self.real_robot:
                angle = scan.angle_min + scan.angle_increment * min_idx  # Kobuki simulator has forward-facing laser
            else:
                # Laser faces backward: add pi (180°)
                # Laser upside down: flip angle (multiply by -1)
                angle = -(scan.angle_min + scan.angle_increment * min_idx) + math.pi
            
            angle_deg = math.degrees(angle)

            self.get_logger().info('Obstacle at {:.2f} m, angle {:.2f} deg'.format(distance_min, angle_deg))
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
