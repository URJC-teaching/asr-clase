# Copyright 2025 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
import math

class VFFControllerNode(Node):
    def __init__(self):
        super().__init__('vff_controller_node')

        # Parameters
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('attractive_weight', 1.0)
        self.declare_parameter('repulsive_weight', 1.0)
        self.declare_parameter('stay_distance', 0.5)

        self.max_speed = self.get_parameter('max_speed').value
        self.attractive_weight = self.get_parameter('attractive_weight').value
        self.repulsive_weight = self.get_parameter('repulsive_weight').value
        self.stay_distance = self.get_parameter('stay_distance').value

        # Subscribers
        self.attractive_sub = self.create_subscription(
            Vector3,
            'attractive_vector',
            self.attractive_callback,
            10
        )

        self.repulsive_sub = self.create_subscription(
            Vector3,
            'repulsive_vector',
            self.repulsive_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'vel', 10)

        # Internal state
        self.attractive_vec = Vector3()
        self.repulsive_vec = Vector3()

    def attractive_callback(self, msg: Vector3):
        self.attractive_vec = msg
        self.compute_and_publish_cmd()

    def repulsive_callback(self, msg: Vector3):
        self.repulsive_vec = msg
        self.compute_and_publish_cmd()

    def compute_and_publish_cmd(self):

        vff_x = self.attractive_weight * self.attractive_vec.x - self.repulsive_weight * self.repulsive_vec.x
        vff_y = self.attractive_weight * self.attractive_vec.y - self.repulsive_weight * self.repulsive_vec.y

        # Convert to speed and heading
        angle = math.atan2(vff_y, vff_x)
        speed = min(self.max_speed, math.hypot(vff_x, vff_y))

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = angle  # simple proportional

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Cmd: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}')

        # Reset vectors after publishing
        self.attractive_vec = Vector3()
        self.repulsive_vec = Vector3()

def main(args=None):
    rclpy.init(args=args)
    node = VFFControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
