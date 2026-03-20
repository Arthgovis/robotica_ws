from webbrowser import get
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import sys

class SrvClient(Node):
    def __init__(self):
        super().__init__('start_patrol_client')
        self.srv_client = self.create_client(SetBool, 'start_patrol')
        while not self.srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, data):
        request = SetBool.Request()
        request.data = data
        future = self.srv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    srv_client = SrvClient()
    result = srv_client.send_request(True)
    srv_client.get_logger().info(f'Service response: {result.success}, {result.message}')
    srv_client.destroy_node()
    rclpy.shutdown()