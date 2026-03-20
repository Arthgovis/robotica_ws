import rclpy
from rclpy.node import Node
from custom_interfaces.srv import Aula9
import sys

class SrvClient(Node):
    def __init__(self):
        super().__init__('aula9_srv_client')
        self.srv_client = self.create_client(Aula9, 'aula9_srv')
        while not self.srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        request = Aula9.Request()
        request.a = a
        request.b = b
        future = self.srv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    srv_client = SrvClient()
    result = srv_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    srv_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), result.sum))
    srv_client.destroy_node()
    rclpy.shutdown()