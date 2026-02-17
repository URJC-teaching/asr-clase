import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion


class TFSquareMover(Node):
    def __init__(self):
        super().__init__('tf_square_mover')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.control_loop)

        self.state = 'init'
        self.odom2blref = None  # Guardamos odom2blref (base_link referencia a odom)
        self.side_count = 0

    def transform_to_matrix(self, transform_stamped):
        """Convierte TransformStamped a matriz de transformación 2D (3x3)"""
        t = transform_stamped.transform
        x = t.translation.x
        y = t.translation.y
        q = t.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        matrix = np.array([
            [cos_yaw, -sin_yaw, x],
            [sin_yaw,  cos_yaw, y],
            [0,        0,       1]
        ])
        return matrix
    
    def matrix_to_pose(self, matrix):
        """Extrae x, y, yaw de una matriz de transformación 2D (3x3)"""
        x = matrix[0, 2]
        y = matrix[1, 2]
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        return x, y, yaw

    def control_loop(self):
        try:
            odom2bl = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Inicializar la referencia si es la primera vez
        if self.odom2blref is None:
            self.odom2blref = odom2bl
            self.get_logger().info("Initial reference frame captured")

        if self.state == 'init':
            # Guardar la transformación de referencia para este lado
            self.odom2blref = odom2bl
            self.state = 'forward'
            self.get_logger().info(f"Starting side {self.side_count + 1}")
            return
            

        elif self.state == 'forward':
            # blref2bl = blef2odom @ odom2bl
            # blref2bl = inv(odom2blref) @ odom2bl
            T_odom2blref = self.transform_to_matrix(self.odom2blref)
            T_odom2bl = self.transform_to_matrix(odom2bl)
            T_blref2bl = np.linalg.inv(T_odom2blref) @ T_odom2bl
            
            x, y, _ = self.matrix_to_pose(T_blref2bl)
            distance = math.sqrt(x**2 + y**2)
            self.get_logger().info(f"Moving forward on side {self.side_count + 1}. distance: {distance:.2f}")
            
            if distance < 1.0:  # move 1 meter
                twist = Twist()
                twist.linear.x = 0.5
                self.publisher.publish(twist)
            else:
                self.publisher.publish(Twist())  # stop
                self.state = 'turn'
                self.odom2blref = odom2bl  # Nueva referencia para el giro
                time.sleep(0.5)
            

        elif self.state == 'turn':
            # Calcular blref2bl = inv(odom2blref) @ odom2bl
            T_odom2blref = self.transform_to_matrix(self.odom2blref)
            T_odom2bl = self.transform_to_matrix(odom2bl)
            T_blref2bl = np.linalg.inv(T_odom2blref) @ T_odom2bl
            
            _, _, yaw = self.matrix_to_pose(T_blref2bl)
            self.get_logger().info(f"Turning at side {self.side_count + 1}. angle: {math.degrees(yaw):.2f} deg")

            if abs(yaw) < math.pi / 2:
                twist = Twist()
                twist.angular.z = 1.0
                self.publisher.publish(twist)
            else:
                self.publisher.publish(Twist())  # stop
                self.side_count += 1
                if self.side_count >= 4:
                    self.get_logger().info("Finished square.")
                    self.state = 'done'
                else:
                    self.state = 'init'
                time.sleep(0.5)
            

        elif self.state == 'done':
            self.publisher.publish(Twist())

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TFSquareMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
