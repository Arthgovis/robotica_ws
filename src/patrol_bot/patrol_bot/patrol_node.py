import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import math
from custom_interfaces.action import RotateAngle

class Patrol(Node):
    def __init__(self):
        super().__init__('Patrol_bot_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.subscription_callback, 10)
        self.min_range = float('inf')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start_patrol = self.create_service(SetBool, 'start_patrol', self.start_patrol_callback)
        self.rotate_client = ActionClient(self, RotateAngle, 'rotate_angle_action')
        self.navegando = False
        self.girando = False
        self.tempo_giro = 0.0
        self.vel = Twist()


    def start_patrol_callback(self, request, response):
        if self.navegando == False:
            self.timer = self.create_timer(0.1, self.move_callback)
            self.navegando = True
            response.success = True
            response.message = "Patrulha iniciada"
            return response
        
        elif self.navegando == True:
            response.success = False
            response.message = "Robô já está em operação"
            return response

    def subscription_callback(self, msg):
        front_index = len(msg.ranges) // 2
        window = msg.ranges[front_index-5:front_index+5]
        valid = [r for r in window if not math.isinf(r) and not math.isnan(r)]
        if valid:
            self.min_range = min(valid)

    def move_callback(self):
        # Keep patrol paused while rotate action controls /cmd_vel.
        if self.girando:
            return

        self.vel.linear.x = 0.5
        self.vel.angular.z = 0.0

        if self.min_range < 1.0:
            self.vel.linear.x = 0.0
            self.publisher.publish(self.vel)
            self.send_rotate_goal(90.0)
            return

        self.publisher.publish(self.vel)

    def send_rotate_goal(self, degrees):
        if self.girando:
            return
        if not self.rotate_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('Rotate action server not available.')
            return

        goal_msg = RotateAngle.Goal()
        goal_msg.degrees = float(degrees)
        self.girando = True
        send_goal_future = self.rotate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.rotate_feedback_callback,
        )
        send_goal_future.add_done_callback(self.rotate_goal_response_callback)

    def rotate_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Rotate goal rejected.')
            self.girando = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.rotate_result_callback)

    def rotate_feedback_callback(self, feedback_msg):
        degrees_remaining = feedback_msg.feedback.degrees_remaining
        self.get_logger().info(f'Rotate feedback: {degrees_remaining:.2f} deg remaining')

    def rotate_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Rotate goal completed.')
        else:
            self.get_logger().warn('Rotate goal failed.')
        self.girando = False


def main(args=None):
    rclpy.init(args=args)
    patrol = Patrol()
    rclpy.spin(patrol)
    patrol.destroy_node()
    rclpy.shutdown()