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
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        # Parameter: minimum distance to consider obstacle
        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('base_frame', 'base_footprint')

        self.min_distance = self.get_parameter('min_distance').value
        self.base_frame = self.get_parameter('base_frame').value

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def laser_callback(self, scan: LaserScan):
        if not scan.ranges:
            return
        
        ranges = [r if math.isfinite(r) else float('inf') for r in scan.ranges] # all NaN to inf so they are ignored by min()
        if not ranges:
            self.get_logger().debug('No valid laser measurements after filtering')
            return
        
        distance_min = min(ranges)
        min_idx = ranges.index(distance_min)

        if distance_min <= self.min_distance:

            angle = scan.angle_min + scan.angle_increment * min_idx # relative to the laser frame
            x = distance_min * math.cos(angle)
            y = distance_min * math.sin(angle)

            pt = PointStamped()
            pt.header = scan.header
            pt.point.x = x
            pt.point.y = y
            pt.point.z = 0.0

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    scan.header.frame_id,
                    rclpy.time.Time()
                )
                pt_base = do_transform_point(pt, transform)

                angle_base = math.atan2(pt_base.point.y, pt_base.point.x)
                distance_base = math.hypot(pt_base.point.x, pt_base.point.y)
                self.get_logger().info(
                    f'Obstacle @ {self.base_frame}: ({pt_base.point.x:.2f}, {pt_base.point.y:.2f}); '
                    f'd={distance_base:.2f} m, a={math.degrees(angle_base):.2f} deg'
                )

                # if (abs(angle_base) > math.pi/2):
                #     return

                self.publish_repulsive_vector(distance_base, angle_base)

            except Exception as e:
                self.get_logger().warn(f'No TF from {scan.header.frame_id} to {self.base_frame}: {e}')


            
        else:
            self.get_logger().debug(f'No obstacle closer than {self.min_distance:.2f} m (min={distance_min:.2f} m)')           

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
