import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from comms_interfaces.action import GenerateInformation
import time

class GenerateServer(Node):
    def __init__(self):
        super().__init__('generate_information_server')
        self._action_server = ActionServer(
            self,
            GenerateInformation,
            'generate_information',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received key: {goal_handle.request.key}')
        feedback_msg = GenerateInformation.Feedback()

        # Simulamos la generación del contenido en pasos
        content = ""
        for i in range(3):
            content += f"[fragment {i}] "
            feedback_msg.provisional_content = content
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = GenerateInformation.Result()
        result.success = True
        result.final_content = f"Final content for key '{goal_handle.request.key}': {content.strip()}"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = GenerateServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
