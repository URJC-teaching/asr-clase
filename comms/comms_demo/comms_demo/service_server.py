from comms_interfaces.srv import GetInformation
import rclpy
from rclpy.node import Node

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.srv = self.create_service(GetInformation, 'get_information', self.handle_request)

    def handle_request(self, request, response):
        response.content = f"Contenido para clave: {request.key}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()