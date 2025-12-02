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
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('repulsive_gain_factor', 1.0)
        self.declare_parameter('repulsive_influence_distance', 0.5)
        self.declare_parameter('stay_distance', -1.0) # -1.0 means no stay distance (2D case)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.repulsive_gain_factor = self.get_parameter('repulsive_gain_factor').value
        self.repulsive_influence_distance = self.get_parameter('repulsive_influence_distance').value
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
        self.get_logger().debug(f'Received Attractive vector: x={msg.x:.2f}, y={msg.y:.2f}. Magnitude={math.hypot(msg.x, msg.y):.2f}. Angle={math.degrees(math.atan2(msg.y, msg.x)):.2f} deg')
        self.compute_and_publish_cmd()

    def repulsive_callback(self, msg: Vector3):
        self.repulsive_vec = msg
        self.get_logger().debug(f'Received Repulsive vector: x={msg.x:.2f}, y={msg.y:.2f}. Magnitude={math.hypot(msg.x, msg.y):.2f}. Angle={math.degrees(math.atan2(msg.y, msg.x)):.2f} deg')
        self.compute_and_publish_cmd()

    def compute_and_publish_cmd(self):

        if self.stay_distance > 0:
            distance = math.hypot(self.attractive_vec.x, self.attractive_vec.y)
            if distance < self.stay_distance:
                self.get_logger().debug(f'Target @ {self.attractive_vec.x:.2f} m, {self.attractive_vec.y:.2f}. '
                                        f'Within stay distance ({distance:.2f} < {self.stay_distance}), ignoring attraction')
                # self.attractive_vec = Vector3() # Zero attractive vector
                return  # Do not move if within stay distance

        obstacle_distance = math.hypot(self.repulsive_vec.x, self.repulsive_vec.y)
        
        # Initialize repulsive force components to zero
        repulsive_force_x = 0.0
        repulsive_force_y = 0.0

        rho_0 = self.repulsive_influence_distance
        avoidance_needed = False
        if 0.0 < obstacle_distance <= rho_0: # If within influence distance
            avoidance_needed = True
            # Normally F_rep proportional to (1/d - 1/rho_0) but we kee it simple here

            force_mag_gain = 1.0 / obstacle_distance**2 # The closer the obstacle, the stronger the repulsive force
     
            unit_x = self.repulsive_vec.x / obstacle_distance
            unit_y = self.repulsive_vec.y / obstacle_distance

            repulsive_force_x = self.repulsive_gain_factor * force_mag_gain * unit_x
            repulsive_force_y = self.repulsive_gain_factor * force_mag_gain * unit_y

            self.get_logger().debug(f'Repulsive magnitude={math.hypot(repulsive_force_x, repulsive_force_y):.2f}. Angle={math.degrees(math.atan2(self.repulsive_vec.y, self.repulsive_vec.x)):.2f} deg')


        vff_x = self.attractive_vec.x - repulsive_force_x
        vff_y = self.attractive_vec.y - repulsive_force_y

        self.get_logger().info(f'VFF vector: x={vff_x:.2f}, y={vff_y:.2f}. Magnitude={math.hypot(vff_x, vff_y):.2f} Angle={math.degrees(math.atan2(vff_y, vff_x)):.2f} deg')

        angle = math.atan2(vff_y, vff_x)

        cmd = Twist()
        cmd.linear.x = min(self.max_linear_speed, math.hypot(vff_x, vff_y))

        rotation_dir = 1.0 if angle >= 0 else -1.0 
        # cmd.angular.z = rotation_dir * self.max_angular_speed

        # ANGULAR_KP = 1.5 
        # proportional_angular_speed = ANGULAR_KP * angle
        # cmd.angular.z = max(min(angle, self.max_angular_speed), -self.max_angular_speed)
        cmd.angular.z = rotation_dir * min(self.max_angular_speed, abs(angle))

        self.cmd_pub.publish(cmd)
        self.get_logger().debug(f'Cmd: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}')

        # This hack is needed when the obstacle is detected once, but then not detected anymore
        # if avoidance_needed:
        #     start_time = self.get_clock().now()
        #     period = 0.1

        #     def _hold_publish():
        #         now = self.get_clock().now()
        #         elapsed = (now - start_time).nanoseconds / 1e9
        #         if elapsed < 0.5:
        #             self.cmd_pub.publish(cmd)
        #             self.get_logger().debug(f'Cmd: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}')
        #         else:
        #             timer.cancel()

        #     timer = self.create_timer(period, _hold_publish)

        # Reset vectors after publishing
        self.attractive_vec = Vector3()
        self.repulsive_vec = Vector3()

def main(args=None):
    
    rclpy.init(args=args)
    node = VFFControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
