import rclpy
from rclpy.node import Node
from custom_interfaces.srv import Aula9

class SrvServer(Node):
    def __init__(self):
        super().__init__('aula9_srv_server')
        self.srv_server = self.create_service(Aula9, 'aula9_srv', self.srv_callback)

    def srv_callback(self, request, response):
        response.sum = request.a + request.b
        return response

def main(args=None):
    rclpy.init(args=args)
    srv_server = SrvServer()
    rclpy.spin(srv_server)
    node.destroy_node()
    rclpy.shutdown()