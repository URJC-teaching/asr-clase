import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import py_trees
import py_trees.behaviour
import py_trees.composites
import py_trees.common
import py_trees_ros

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, '/out_vel', 10)

    def update(self):
        msg = Twist()
        msg.linear.x = 0.2
        self.cmd_pub.publish(msg)
        return py_trees.common.Status.SUCCESS

class CheckBump(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bumped = False
        node.create_subscription(Bool, '/bump', self.bump_callback, 10)

    def bump_callback(self, msg):
        self.bumped = msg.data

    def update(self):
        if self.bumped:
            self.bumped = False
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class BackOff(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, '/out_vel', 10)
        self.counter = 0

    def initialise(self):
        self.counter = 0

    def update(self):
        if self.counter < 20:
            msg = Twist()
            msg.linear.x = -0.2
            self.cmd_pub.publish(msg)
            self.counter += 1
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

class Turn(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, '/out_vel', 10)
        self.counter = 0

    def initialise(self):
        self.counter = 0

    def update(self):
        if self.counter < 30:
            msg = Twist()
            msg.angular.z = 0.5
            self.cmd_pub.publish(msg)
            self.counter += 1
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

def create_tree(node):
    check_bump = CheckBump("Check Bump", node)
    back_off = BackOff("Back Off", node)
    turn = Turn("Turn", node)
    move_forward = MoveForward("Move Forward", node)

    react_to_bump = py_trees.composites.Sequence("React To Bump", memory=False)
    
    react_to_bump.add_children([check_bump, back_off, turn])

    root = py_trees.composites.Selector("Bump-Go Root", memory=False)
    root.add_children([react_to_bump, move_forward])
    return root

class BumpGoNode(Node):
    def __init__(self):
        super().__init__('bump_go')
        self.tree = py_trees_ros.trees.BehaviourTree(create_tree(self))
        self.tree.setup(timeout=15)

def main(args=None):
    rclpy.init(args=args)
    node = BumpGoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()