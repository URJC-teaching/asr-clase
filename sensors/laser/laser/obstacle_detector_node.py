# Copyright 2025 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('base_frame', 'base_footprint')

        self.min_distance = self.get_parameter('min_distance').value
        self.base_frame = self.get_parameter('base_frame').value

        self.get_logger().info(f'Obstacle_detector_node set to {self.min_distance:.2f} m')

        self.laser_sub = self.create_subscription(
            LaserScan,
            'input_laser',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.obstacle_pub = self.create_publisher(Bool, 'obstacle', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def laser_callback(self, scan: LaserScan):
        if not scan.ranges:
            return
        
        ranges = [r if math.isfinite(r) else float('inf') for r in scan.ranges] # all NaN to inf so they are ignored by min()
        if not ranges:
            self.get_logger().debug('No valid laser measurements after filtering')
            return
        
        distance_min = min(ranges) # closest obstacle
        min_idx = ranges.index(distance_min)

        obstacle_msg = Bool()
        obstacle_msg.data = (distance_min < self.min_distance)
        self.obstacle_pub.publish(obstacle_msg)

        if obstacle_msg.data: # if obstacle detected within min_distance
            angle = scan.angle_min + scan.angle_increment * min_idx # relative to the sensor frame
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
                ) # get latest available transform between the laser frame and the base frame
                pt_base = do_transform_point(pt, transform) # transform point to base frame

                angle_base = math.atan2(pt_base.point.y, pt_base.point.x)
                distance_base = math.hypot(pt_base.point.x, pt_base.point.y)
                self.get_logger().info(
                    f'Obstacle @ {self.base_frame}: x={pt_base.point.x:.2f}, y={pt_base.point.y:.2f}, '
                    f'distance={distance_base:.2f} m, angle={math.degrees(angle_base):.2f} deg'
                )

            except Exception as e:
                self.get_logger().warn(f'No TF from {scan.header.frame_id} to {self.base_frame}: {e}')


            
        else:
            self.get_logger().debug(f'No obstacle closer than {self.min_distance:.2f} m (min={distance_min:.2f} m)')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
