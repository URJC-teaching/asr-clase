import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(2)  # Wait for the publisher to be ready
        self.move_square()

    def move_square(self):
        move_cmd = Twist()
        turn_cmd = Twist()

        move_cmd.linear.x = 0.2      # Forward speed
        turn_cmd.angular.z = 0.5     # Turning speed

        move_duration = 3.0          # Move forward time in seconds
        turn_duration = 1.6          # Turn 90º (approximate)

        for i in range(4):
            self.get_logger().info(f"Side {i+1}: Moving forward")
            self.publisher_.publish(move_cmd)
            time.sleep(move_duration)

            self.get_logger().info(f"Side {i+1}: Turning")
            self.publisher_.publish(turn_cmd)
            time.sleep(turn_duration)

        # Stop the robot
        self.get_logger().info("Square complete. Stopping.")
        self.publisher_.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = SquareMover()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
