# Copyright 2025 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        # Parameter: minimum distance to consider obstacle
        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('real_robot', False)

        self.min_distance = self.get_parameter('min_distance').value
        self.real_robot = self.get_parameter('real_robot').value

        self.get_logger().info(f'ObstacleDetectorNode min_distance={self.min_distance}')

        # Publisher for raw repulsive vector
        self.repulsive_vector_pub = self.create_publisher(Vector3, 'repulsive_vector', 10)

        # Laser subscriber
        self.laser_sub = self.create_subscription(
            LaserScan,
            'input_laser',
            self.laser_callback,
            10
        )

    def laser_callback(self, scan: LaserScan):
        if not scan.ranges:
            return

        # Closest obstacle
        min_idx = min(range(len(scan.ranges)), key=lambda i: scan.ranges[i])
        distance_min = scan.ranges[min_idx]
        self.get_logger().debug(f'Closest obstacle at distance {distance_min:.2f} m')

        if distance_min <= self.min_distance:

            if not self.real_robot:
                angle = scan.angle_min + scan.angle_increment * min_idx
            else:
                # Laser faces backward: add pi (180°)
                # Laser upside down: flip angle (multiply by -1)
                angle = -(scan.angle_min + scan.angle_increment * min_idx) + math.pi
            
            angle_deg = math.degrees(angle)
   
            self.get_logger().info('Obstacle at {:.2f} m, angle {:.2f} deg'.format(distance_min, angle_deg))

            self.publish_repulsive_vector(distance_min, angle)

    def publish_repulsive_vector(self, distance: float, angle: float):
        # Convert polar to Cartesian
        # x is forward, y is left; angle=0 is in front, negative is right, positive is left
        x = math.cos(angle) * distance
        y = math.sin(angle) * distance

        vec = Vector3()
        vec.x = x
        vec.y = y
        vec.z = 0.0

        self.repulsive_vector_pub.publish(vec)
        self.get_logger().debug(f'Repulsive vector x={x:.3f}, y={y:.3f}. d={math.hypot(x, y):.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
