import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.timer import Timer

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Movement Parameters
        self.forward_speed = 0.2
        self.turning_speed = 0.5
        self.move_duration = 5.0      # Forward time for 1 meter (approx.)
        self.turn_duration = 3.14     # Turn time for 90 degrees (approx.)

        self.step_index = 0           
        self.total_steps = 8          # 4 sides * 2 actions per side = 8 steps
        self.sides_completed = 0      # Track number of sides completed
        
        # Time tracking
        self.start_time = self.get_clock().now()

        # Timer setup (runs every 0.1 seconds, 10 Hz)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("SquareMover node initialized. Starting movement sequence...")


    def control_loop(self):
  
        if self.step_index >= self.total_steps:
            # Stop the robot and the timer
            self.publisher_.publish(Twist()) # Publish zero velocities to stop
            self.get_logger().info("Square complete. Stopping robot and shutting down timer.")
            self.timer.cancel() 
            return

        move_cmd = Twist()
        action_name = ""
        current_duration = 0.0

        if self.step_index % 2 == 0:  # Even index (0, 2, 4, 6) means move forward
            move_cmd.linear.x = self.forward_speed
            current_duration = self.move_duration
            action_name = "Moving Forward"
        else:  # Odd index (1, 3, 5, 7) means turn
            move_cmd.angular.z = self.turning_speed
            current_duration = self.turn_duration
            action_name = "Turning"

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.publisher_.publish(move_cmd)

        if elapsed_time >= current_duration:
            self.get_logger().info(f"{action_name} step complete.")
            
            self.step_index += 1  # Move to the next step
            self.start_time = self.get_clock().now()  # Reset the timer for the new step
            
            if self.step_index >= self.total_steps:
                self.get_logger().info("Sequence complete. Waiting for final stop.")


def main(args=None):
    rclpy.init(args=args)
    square_mover = SquareMover()
    
    # rclpy.spin() is necessary to allow the timer to execute repeatedly
    rclpy.spin(square_mover) 
    
    # Cleanup
    square_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()