import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import math
from custom_interfaces.action import RotateAngle

class Patrol(Node):
    def __init__(self):
        super().__init__('Patrol_bot_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.subscription_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start_patrol = self.create_service(SetBool, 'start_client', self.start_patrol_callback)
        self.rotate_client = ActionClient(self, RotateAngle, 'rotate_angle_action')
        self.timer = None
        self.vel = Twist()
        self.reset_patrol_state()

    def reset_patrol_state(self):
        self.min_range = float('inf')
        self.current_x = 0.0
        self.current_y = 0.0
        self.first_rotation = False
        self.odom_received = False
        self.navegando = False
        self.girando = False
        self.stop_after_rotation = False
        self.tempo_giro = 0.0
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

    def set_idle_state(self):
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        self.reset_patrol_state()


    def start_patrol_callback(self, request, response):
        if self.navegando == False:
            if self.timer is None:
                self.timer = self.create_timer(0.1, self.move_callback)
            else:
                self.timer.reset()
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
        window = msg.ranges[front_index-10:front_index+10]
        valid = [r for r in window if not math.isinf(r) and not math.isnan(r)]
        if valid:
            self.min_range = min(valid)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_x = position.x
        self.current_y = position.y
        self.odom_received = True

    def move_callback(self):
        if not self.navegando or self.girando:
            return

        self.vel.linear.x = 1.0
        self.vel.angular.z = 0.0

        if self.min_range < 1.0:
            self.vel.linear.x = 0.0
            self.publisher.publish(self.vel)
            self.stop_after_rotation = False
            self.send_rotate_goal(184.5)
            return
        elif self.first_rotation and self.current_x <= 0.02:
            self.vel.linear.x = 0.0
            self.publisher.publish(self.vel)
            self.stop_after_rotation = True
            self.send_rotate_goal(184.5)
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
            self.stop_after_rotation = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.rotate_result_callback)

    def rotate_feedback_callback(self, feedback_msg):
        degrees_remaining = feedback_msg.feedback.degrees_remaining
        self.get_logger().info(f'Rotate feedback: {degrees_remaining:.2f} deg remaining')

    def rotate_result_callback(self, future):
        result = future.result().result
        if self.stop_after_rotation:
            if result.success:
                self.get_logger().info('Rotate goal completed.')
            else:
                self.get_logger().warn('Rotate goal failed. Returning to IDLE because patrol is back near the start point.')
            self.set_idle_state()
            self.get_logger().info('Patrol returned to IDLE and is waiting for a new start command.')
            return
        
        self.first_rotation = True
        self.stop_after_rotation = False
        self.girando = False


def main(args=None):
    rclpy.init(args=args)
    patrol = Patrol()
    rclpy.spin(patrol)
    patrol.destroy_node()
    rclpy.shutdown()