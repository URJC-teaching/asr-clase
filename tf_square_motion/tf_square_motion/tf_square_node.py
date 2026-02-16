import math
import time

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
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.side_count = 0

    def control_loop(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

            # Get current pose
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        if self.state == 'init':
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
            self.state = 'forward'
            self.get_logger().info(f"Starting side {self.side_count + 1}")
            return

        elif self.state == 'forward':
            dx = x - self.start_x
            dy = y - self.start_y
            distance = math.sqrt(dx**2 + dy**2)
            self.get_logger().info(f"Moving forward on side {self.side_count + 1}. distance: {distance:.2f}")

            if distance < 1.0:  # move 1 meter
                twist = Twist()
                twist.linear.x = 0.5
                self.publisher.publish(twist)
            else:
                self.publisher.publish(Twist())  # stop
                self.state = 'turn'
                self.start_yaw = yaw
                time.sleep(0.5)

        elif self.state == 'turn':
            # Compute angle turned
            yaw_diff = self.normalize_angle(yaw - self.start_yaw)
            self.get_logger().info(f"Turning at side {self.side_count + 1}. angle: {math.degrees(yaw_diff):.2f} deg")

            if abs(yaw_diff) < math.pi / 2:
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
