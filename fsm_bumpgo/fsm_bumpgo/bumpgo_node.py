import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import BumperEvent

from enum import IntEnum
from rclpy.duration import Duration

import time


class State(IntEnum):
    FORWARD = 0
    BACK = 1
    TURN = 2

SPEED_LINEAR = 0.2
SPEED_ANGULAR = 1.0
BACKING_TIME = Duration(seconds=2.0)
TURNING_TIME = Duration(seconds=2.0)


class BumpGoNode(Node):

    def __init__(self):
        super().__init__('bump_go')

        self.state = State.FORWARD
        self.state_ts = self.get_clock().now()

        self.last_bump = BumperEvent()
        self.last_bump.state = BumperEvent.RELEASED

        self.bumper_sub = self.create_subscription(
            BumperEvent,
            '/bumper',
            self.bumper_callback,
            10
        )

        self.vel_pub = self.create_publisher(Twist, '/out_vel', 10)

        self.timer = self.create_timer(0.05, self.control_cycle)

    def bumper_callback(self, msg):
        self.last_bump = msg

    def control_cycle(self):
        out_vel = Twist()
    
        
        if self.state == State.FORWARD:
            self.get_logger().info('FORWARD')
            out_vel.linear.x = SPEED_LINEAR
            if self.check_forward_2_back():
                self.go_state(State.BACK)

        elif self.state == State.BACK:
            self.get_logger().info('BACK')
            out_vel.linear.x = -SPEED_LINEAR
            if self.check_back_2_turn():
                self.go_state(State.TURN)

        elif self.state == State.TURN:
            self.get_logger().info('TURN')
            out_vel.angular.z = SPEED_ANGULAR
            if self.check_turn_2_forward():
                self.go_state(State.FORWARD)

        self.vel_pub.publish(out_vel)

    def go_state(self, new_state):
        self.state = new_state
        self.state_ts = self.get_clock().now()

    def check_forward_2_back(self):
        return self.last_bump.state == BumperEvent.PRESSED

    def check_back_2_turn(self):
        return (self.get_clock().now() - self.state_ts) > BACKING_TIME

    def check_turn_2_forward(self):
        return (self.get_clock().now() - self.state_ts) > TURNING_TIME


def main(args=None):
    rclpy.init(args=args)
    node = BumpGoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()