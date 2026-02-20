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

        self.declare_parameter('erratic', False)
        self.erratic = self.get_parameter('erratic').get_parameter_value().bool_value

        self.get_logger().info(f"TFSeekerNode initialized with erratic={self.erratic}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        if not self.erratic:
            # PID con constantes ajustadas para evitar sobrepaso y oscilación
            # kp bajo para evitar sobrereacción, ki muy bajo o cero para evitar acumulación, kd para amortiguamiento
            self.vlin_pid = PIDController(0.0, 1.0, -0.5, 0.5, kp=0.3, ki=0.0, kd=0.15)
            self.vrot_pid = PIDController(0.0, 1.0, -0.5, 0.5, kp=0.6, ki=0.0, kd=0.25)
        else:
            # Constantes del PID mal ajustadas a propósito para que el robot oscile un poco y no se quede estático
            # kp muy alto causa sobrereacción, ki causa acumulación y sobrepaso, kd bajo no estabiliza
            self.vlin_pid = PIDController(0.0, 1.0, -0.5, 0.5, kp=5.0, ki=1.5, kd=0.0)
            self.vrot_pid = PIDController(0.0, 1.0, -0.5, 0.5, kp=8.0, ki=1.0, kd=0.0)
        

        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.control_cycle)

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

            # Pasar dt correcto a los PIDs (0.05 segundos = período del timer)
            vel_rot = max(-2.0, min(self.vrot_pid.get_output(angle, self.timer_period), 2.0))
            vel_lin = max(-1.0, min(self.vlin_pid.get_output(dist - 1.0, self.timer_period), 1.0))

            self.get_logger().debug(f'Angle error: {angle:.2f}. Angular speed: {vel_rot:.2f}')
            self.get_logger().info(f'Distance error: {dist - 1:.2f}. Linear speed: {vel_lin:.2f}')

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
