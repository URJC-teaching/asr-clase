# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

from tf2_geometry_msgs import do_transform_pose
from tf2_ros.transform_broadcaster import TransformBroadcaster

from .pid_controller import PIDController  # Asegúrate de que esté en el mismo paquete

class TFSeekerNode(Node):

    def __init__(self):
        super().__init__('tf_seeker')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.vlin_pid = PIDController(0.0, 1.0, 0.0, 0.7)
        self.vrot_pid = PIDController(0.0, 1.0, 0.3, 1.0)

        self.timer = self.create_timer(0.05, self.control_cycle)

    def control_cycle(self):

        # Check if the transform is available
        if not self.tf_buffer.can_transform('base_footprint', 'target', rclpy.time.Time()):
            self.get_logger().warn('Waiting for transform base_footprint -> target')
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint', 'target', rclpy.time.Time())

            x = tf.transform.translation.x
            y = tf.transform.translation.y

            angle = math.atan2(y, x)
            dist = math.sqrt(x ** 2 + y ** 2)

            vel_rot = max(-2.0, min(self.vrot_pid.get_output(angle), 2.0))
            vel_lin = max(-1.0, min(self.vlin_pid.get_output(dist - 1.0), 1.0))

            twist = Twist()
            twist.linear.x = vel_lin
            twist.angular.z = vel_rot

            self.vel_publisher.publish(twist)

            if abs(angle) < 0.2 and dist < 1.3:
                self.get_logger().info('Pew Pew Madafakas')

        except Exception as e:
            self.get_logger().warn(f'Error in TF base_footprint -> target: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = TFSeekerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
