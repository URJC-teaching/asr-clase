from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import BumperEvent

import py_trees
import py_trees.behaviour
import py_trees.composites
import py_trees.common
from py_trees.blackboard import Client

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Client(name=name)
        self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
        self.cmd_pub = None

    def setup(self, **kwargs):
        node = self.blackboard.node
        self.cmd_pub = node.create_publisher(Twist, '/out_vel', 10)

    def update(self):
        self.blackboard.node.get_logger().info("Moving forward...")
        msg = Twist()
        msg.linear.x = 0.2
        self.cmd_pub.publish(msg)
        return py_trees.common.Status.RUNNING

class CheckBump(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Client(name=name)
        self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
        self.bumped = False
        self.sub = None

    def setup(self, **kwargs):
        node = self.blackboard.node
        self.sub = node.create_subscription(BumperEvent, '/bumper', self.bumper_callback, 10)

    def bumper_callback(self, msg):
        if msg.state == BumperEvent.PRESSED:
            self.blackboard.node.get_logger().info("Bumper pressed!")
            self.bumped = True

    def update(self):
        if self.bumped:
            self.bumped = False
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class BackOff(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Client(name=name)
        self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
        self.cmd_pub = None
        self.start_time = None
        self.duration_sec = 2.0

    def setup(self, **kwargs):
        node = self.blackboard.node
        self.cmd_pub = node.create_publisher(Twist, '/out_vel', 10)

    def initialise(self):
        self.start_time = None

    def update(self):
        self.blackboard.node.get_logger().info("Backing off...")
        node = self.blackboard.node

        if self.start_time is None:
            self.start_time = node.get_clock().now()

        now = node.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed < self.duration_sec:
            msg = Twist()
            msg.linear.x = -0.2
            self.cmd_pub.publish(msg)
            return py_trees.common.Status.RUNNING
        else:
            self.start_time = None
            stop = Twist()
            self.cmd_pub.publish(stop)
            return py_trees.common.Status.SUCCESS

class Turn(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Client(name=name)
        self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
        self.cmd_pub = None
        self.duration_sec = 2.0

    def setup(self, **kwargs):
        node = self.blackboard.node
        self.cmd_pub = node.create_publisher(Twist, '/out_vel', 10)

    def initialise(self):
        self.start_time = None

    def update(self):
        self.blackboard.node.get_logger().info("Turning...")
        node = self.blackboard.node
        
        if self.start_time is None:
            self.start_time = node.get_clock().now()
        
        now = node.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed < self.duration_sec:
            msg = Twist()
            msg.angular.z = 0.5
            self.cmd_pub.publish(msg)
            return py_trees.common.Status.RUNNING
        else:
            self.start_time = None
            stop = Twist()
            self.cmd_pub.publish(stop)
            return py_trees.common.Status.SUCCESS
        
class BumpGoBT():
    def create_tree(self):
        check_bump = CheckBump("check_bump")
        back_off = BackOff("back_off")
        turn = Turn("turn")
        move_forward = MoveForward("forward")

        invert_bump = py_trees.decorators.Inverter("invert_bump", child=check_bump)
        
        back_and_turn = py_trees.composites.Sequence("back_and_turn", memory=True)
        back_and_turn.add_children([back_off, turn])
        
        react_to_bump = py_trees.composites.Selector("bumpgo_seq", memory=True)
        react_to_bump.add_children([invert_bump, back_and_turn])

        root = py_trees.composites.Sequence("bumpgo_root", memory=False)
        root.add_children([react_to_bump, move_forward])

        return root

