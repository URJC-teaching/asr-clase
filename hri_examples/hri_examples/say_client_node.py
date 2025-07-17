import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from audio_common_msgs.action import TTS 

from std_msgs.msg import String

class SayClientNode(Node):

    def __init__(self):
        super().__init__('say_client_node')

        # Create the action client for 'say' action
        self._action_client = ActionClient(self, TTS, 'say')

        # Optional: listen for topic or timer to send goal
        # self.create_timer(2.0, self.send_goal)
        self.send_goal()

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('TTS action server not available!')
            return

        # Define the goal
        goal_msg = TTS.Goal()
        goal_msg.text = "Hola, ¿qué tal estáis?"
        
        self.get_logger().info(f'Sending goal: "{goal_msg.text}"')

        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted! Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # result = future.result().result
        # self.get_logger().info(f'Result: {result.success}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SayClientNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
