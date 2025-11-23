import rclpy
from rclpy.node import Node
import py_trees
import py_trees.common
from py_trees.blackboard import Client
from bt_bumpgo.bumpgo_bt import BumpGoBT
import time


def main(args=None):
    rclpy.init(args=args)
    
    # 1. Create a standard ROS node
    ros_node = Node('bump_go')

    # 2. Initialize Blackboard and set the node
    blackboard = Client(name="global_blackboard")
    blackboard.register_key(key="node", access=py_trees.common.Access.WRITE)
    blackboard.node = ros_node

    # 3. Create and Setup the Tree wrapper
    bumpgo = BumpGoBT()
    root = bumpgo.create_tree()

    # OPTION: py_trees_ros (wrapper that manages ticking and ROS integration)
    # tree = py_trees_ros.trees.BehaviourTree(root)
    
    # # Connect the tree to the ROS node
    # tree.setup(node=ros_node, timeout=15)
    
    # # Start the tree ticking
    # tree.tick_tock(period_ms=100.0)

    # try:
    #     rclpy.spin(ros_node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     tree.shutdown()
    #     ros_node.destroy_node()
    #     rclpy.shutdown()

    # OPTION: Manual ticking
    root.setup_with_descendants()
    try:
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)
            root.tick_once()
            if root.status != py_trees.common.Status.RUNNING:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()