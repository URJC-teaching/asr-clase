import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from std_msgs.msg import Int32
from rclpy.timer import Timer
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

class LifeCyclePublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Int32, 'int_topic', 10)
        self.message_ = Int32()
        self.timer_ = None

    def on_configure(self, state: LifecycleState):
        self.get_logger().info(f"[{self.get_name()}] Configuring")
        self.get_logger().info(f"[{self.get_name()}] Configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info(f"[{self.get_name()}] Activating...")
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        # self.publisher_.on_activate()
        self.get_logger().info(f"[{self.get_name()}] Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info('Deactivating...')
        self.timer_ = None
        # self.publisher_.on_deactivate()
        self.get_logger().info(f"[{self.get_name()}] Deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info('Cleaning Up...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info('Shutting Down...')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState):
        self.get_logger().info('Error State')
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        self.message_.data += 1
        self.publisher_.publish(self.message_)

def main(args=None):
    rclpy.init(args=args)
    node = LifeCyclePublisherNode()
    
    node.trigger_configure()
    node.trigger_activate()

    # Spin the node to keep it active
    rclpy.spin(node)

    # Shutdown the node
    node.trigger_deactivate()
    node.trigger_cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
