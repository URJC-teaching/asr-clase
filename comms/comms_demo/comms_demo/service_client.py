from comms_interfaces.srv import GetInformation
import rclpy
from rclpy.node import Node

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.cli = self.create_client(GetInformation, 'get_information')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = GetInformation.Request()

    def send_request(self, key):
        self.req.key = key
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Respuesta recibida: {response.content}')
        except Exception as e:
            self.get_logger().error(f'Error al recibir respuesta: {e}')
        finally:
            rclpy.shutdown()  # Finaliza el nodo tras recibir la respuesta

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    node.send_request('example_key')
    rclpy.spin(node)

if __name__ == '__main__':
    main()
