import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
import threading
from custom_interfaces.action import RotateAngle

class RotateServerNode(Node):
    def __init__(self):
        super().__init__('patrol_rotate_server')
        self.current_yaw = 0.0
        self.odom_received = False
        self.target_yaw = 0.0
        self.state_lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            RotateAngle,
            'rotate_angle_action',
            self.my_action_callback,
            callback_group=self.callback_group,
        )

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self.state_lock:
            self.current_yaw = yaw
            self.odom_received = True

    def my_action_callback(self, goal_handle):
        degrees = goal_handle.request.degrees
        target_rotation = abs(math.radians(degrees))
        direction = 1.0 if degrees >= 0.0 else -1.0

        wait_start = time.time()
        while True:
            with self.state_lock:
                if self.odom_received:
                    initial_yaw = self.current_yaw
                    break
            if time.time() - wait_start > 2.0:
                goal_handle.abort()
                result = RotateAngle.Result()
                result.success = False
                self.get_logger().error('No odometry received before timeout.')
                return result
            time.sleep(0.05)

        self.target_yaw = (initial_yaw + direction * target_rotation + math.pi) % (2 * math.pi) - math.pi

        vel_msg = Twist()
        angular_speed = 0.8 * direction
        tolerance = math.radians(2.0)
        accumulated_rotation = 0.0
        previous_yaw = initial_yaw

        while accumulated_rotation + tolerance < target_rotation:
            if goal_handle.is_cancel_requested:
                vel_msg.angular.z = 0.0
                self.publisher.publish(vel_msg)
                goal_handle.canceled()
                result = RotateAngle.Result()
                result.success = False
                self.get_logger().info('Rotation goal canceled.')
                return result

            with self.state_lock:
                current_yaw = self.current_yaw

            delta_yaw = (current_yaw - previous_yaw + math.pi) % (2 * math.pi) - math.pi
            accumulated_rotation += abs(delta_yaw)
            previous_yaw = current_yaw

            degrees_remaining = max(0.0, math.degrees(target_rotation - accumulated_rotation))

            vel_msg.angular.z = angular_speed
            self.publisher.publish(vel_msg)

            feedback_msg = RotateAngle.Feedback()
            feedback_msg.degrees_remaining = degrees_remaining
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Publishing feedback: {degrees_remaining}')
            time.sleep(0.05)

        vel_msg.angular.z = 0.0
        self.publisher.publish(vel_msg)


        goal_handle.succeed()
        result = RotateAngle.Result()
        result.success = True
        self.get_logger().info('Goal succeeded!')
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = RotateServerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_server)
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()