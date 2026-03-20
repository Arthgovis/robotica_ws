import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interfaces.action import Aula10
import sys

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('aula10_action_client')
        self.action_client = ActionClient(self, Aula10, 'aula10_action')

    def send_goal(self, count_up_to):
        self.get_logger().info('Sending goal...')
        goal_msg = Aula10.Goal()
        goal_msg.count_up_to = count_up_to

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_number}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final Result: {result.final_count}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = ActionClientNode()
    action_client.send_goal(int(sys.argv[1]))
    rclpy.spin(action_client)