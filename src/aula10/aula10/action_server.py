import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_interfaces.action import Aula10
import time

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('aula10_action_server')
        self.action_server = ActionServer(
            self,
            Aula10,
            'aula10_action',
            self.my_action_callback
        )

    def my_action_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        count_up_to = goal_handle.request.count_up_to
        current_number = 0

        while current_number < count_up_to:

            feedback_msg = Aula10.Feedback()
            feedback_msg.current_number = current_number
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Publishing feedback: {current_number}')

            current_number += 1

            time.sleep(1)

        goal_handle.succeed()
        result = Aula10.Result()
        result.final_count = current_number
        self.get_logger().info('Goal succeeded!')
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = ActionServerNode()
    rclpy.spin(action_server)
    rclpy.shutdown()