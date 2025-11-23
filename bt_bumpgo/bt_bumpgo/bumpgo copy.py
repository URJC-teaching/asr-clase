# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Bool

# import py_trees
# import py_trees.behaviour
# import py_trees.composites
# import py_trees.common
# import py_trees_ros
# from py_trees.blackboard import Client
# import time

# class MoveForward(py_trees.behaviour.Behaviour):
#     def __init__(self, name):
#         super().__init__(name)
#         self.blackboard = Client(name=name)
#         self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
#         self.cmd_pub = None

#     def setup(self, **kwargs):
#         node = self.blackboard.node
#         self.cmd_pub = node.create_publisher(Twist, '/out_vel', 10)

#     def update(self):
#         msg = Twist()
#         msg.linear.x = 0.2
#         self.cmd_pub.publish(msg)
#         return py_trees.common.Status.RUNNING

# class CheckBump(py_trees.behaviour.Behaviour):
#     def __init__(self, name):
#         super().__init__(name)
#         self.blackboard = Client(name=name)
#         self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
#         self.bumped = False
#         self.sub = None

#     def setup(self, **kwargs):
#         node = self.blackboard.node
#         self.sub = node.create_subscription(Bool, '/bump', self.bump_callback, 10)

#     def bump_callback(self, msg):
#         if msg.data:
#             self.bumped = True

#     def update(self):
#         if self.bumped:
#             self.bumped = False
#             return py_trees.common.Status.SUCCESS
#         else:
#             return py_trees.common.Status.FAILURE

# class BackOff(py_trees.behaviour.Behaviour):
#     def __init__(self, name):
#         super().__init__(name)
#         self.blackboard = Client(name=name)
#         self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
#         self.cmd_pub = None
#         self.counter = 0

#     def setup(self, **kwargs):
#         node = self.blackboard.node
#         self.cmd_pub = node.create_publisher(Twist, '/out_vel', 10)

#     def initialise(self):
#         self.counter = 0
#         self.blackboard.node.get_logger().info("Backing off...")

#     def update(self):
#         if self.counter < 20:
#             msg = Twist()
#             msg.linear.x = -0.2
#             self.cmd_pub.publish(msg)
#             self.counter += 1
#             return py_trees.common.Status.RUNNING
#         else:
#             return py_trees.common.Status.SUCCESS

# class Turn(py_trees.behaviour.Behaviour):
#     def __init__(self, name):
#         super().__init__(name)
#         self.blackboard = Client(name=name)
#         self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
#         self.cmd_pub = None
#         self.counter = 0

#     def setup(self, **kwargs):
#         node = self.blackboard.node
#         self.cmd_pub = node.create_publisher(Twist, '/out_vel', 10)

#     def initialise(self):
#         self.counter = 0
#         self.blackboard.node.get_logger().info("Turning...")

#     def update(self):
#         if self.counter < 30:
#             msg = Twist()
#             msg.angular.z = 0.5
#             self.cmd_pub.publish(msg)
#             self.counter += 1
#             return py_trees.common.Status.RUNNING
#         else:
#             return py_trees.common.Status.SUCCESS

# def create_tree():
#     check_bump = CheckBump("Check Bump")
#     back_off = BackOff("Back Off")
#     turn = Turn("Turn")
#     move_forward = MoveForward("Move Forward")

#     react_to_bump = py_trees.composites.Sequence("bumpgo_seq", memory=True)
#     react_to_bump.add_children([check_bump, back_off, turn])

#     root = py_trees.composites.Selector("bumpgo_root", memory=False)
#     root.add_children([react_to_bump, move_forward])
#     return root

# def main(args=None):
#     rclpy.init(args=args)
    
#     # 1. Create a standard ROS node
#     ros_node = Node('bump_go')

#     # 2. Initialize Blackboard and set the node
#     blackboard = Client(name="global_blackboard")
#     blackboard.register_key(key="node", access=py_trees.common.Access.WRITE)
#     blackboard.node = ros_node

#     # 3. Create and Setup the Tree wrapper
#     root = create_tree()

#     # OPTION: py_trees_ros (wrapper that manages ticking and ROS integration)
#     # tree = py_trees_ros.trees.BehaviourTree(root)
    
#     # # Connect the tree to the ROS node
#     # tree.setup(node=ros_node, timeout=15)
    
#     # # Start the tree ticking
#     # tree.tick_tock(period_ms=100.0)

#     # try:
#     #     rclpy.spin(ros_node)
#     # except KeyboardInterrupt:
#     #     pass
#     # finally:
#     #     tree.shutdown()
#     #     ros_node.destroy_node()
#     #     rclpy.shutdown()

#     # OPTION: Manual ticking
#     root.setup_with_descendants(timeout=15)
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(ros_node, timeout_sec=0.1)
#             root.tick_once()
#             if root.status != py_trees.common.Status.RUNNING:
#                 break
#             time.sleep(0.5)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         ros_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()