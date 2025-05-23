import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from comms_interfaces.action import GenerateInformation

class GenerateClient(Node):
    def __init__(self):
        super().__init__('generate_information_client')
        self._client = ActionClient(self, GenerateInformation, 'generate_information')

    def send_goal(self, key):
        self._client.wait_for_server()
        goal = GenerateInformation.Goal()
        goal.key = key

        self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)\
            .add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.provisional_content}")

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Success: {result.success}, Final content: {result.final_content}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GenerateClient()
    node.send_goal('example_key')
    rclpy.spin(node)

if __name__ == '__main__':
    main()
