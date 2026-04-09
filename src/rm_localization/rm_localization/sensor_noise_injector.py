import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SensorNoiseInjector(Node):

    def __init__(self):
        super().__init__('sensor_noise_injector')

        self.declare_parameter('noise_x_std', 0.15)
        self.declare_parameter('noise_y_std', 0.15)
        self.declare_parameter('noise_yaw_std', 0.08)
        self.declare_parameter('noise_vx_std', 0.03)
        self.declare_parameter('noise_vyaw_std', 0.2)

        self.create_subscription(Odometry, 'odom_raw', self._odom_cb, 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom_noisy', 10)

        self.get_logger().info('SensorNoiseInjector iniciado.')

    def _odom_cb(self, msg: Odometry):
        p = self.get_parameter
        noise_x    = p('noise_x_std').value
        noise_y    = p('noise_y_std').value
        noise_yaw  = p('noise_yaw_std').value
        noise_vx   = p('noise_vx_std').value
        noise_vyaw = p('noise_vyaw_std').value

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        # --- pose com ruído gaussiano ---
        out.pose.pose.position.x = (
            msg.pose.pose.position.x + np.random.normal(0.0, noise_x)
        )
        out.pose.pose.position.y = (
            msg.pose.pose.position.y + np.random.normal(0.0, noise_y)
        )
        out.pose.pose.position.z = msg.pose.pose.position.z
        out.pose.pose.orientation = msg.pose.pose.orientation

        # covariância da pose (diagonal: x, y, z, roll, pitch, yaw)
        out.pose.covariance = _diag6(
            noise_x**2, noise_y**2, 1e-9,
            1e-9, 1e-9, noise_yaw**2
        )

        # --- twist com ruído gaussiano ---
        out.twist.twist.linear.x = (
            msg.twist.twist.linear.x + np.random.normal(0.0, noise_vx)
        )
        out.twist.twist.linear.y  = msg.twist.twist.linear.y
        out.twist.twist.linear.z  = msg.twist.twist.linear.z
        out.twist.twist.angular.x = msg.twist.twist.angular.x
        out.twist.twist.angular.y = msg.twist.twist.angular.y
        out.twist.twist.angular.z = (
            msg.twist.twist.angular.z + np.random.normal(0.0, noise_vyaw)
        )

        # covariância do twist (diagonal: vx, vy, vz, wx, wy, wz)
        out.twist.covariance = _diag6(
            noise_vx**2, 1e-9, 1e-9,
            1e-9, 1e-9, noise_vyaw**2
        )

        self._odom_pub.publish(out)


def _diag6(v0, v1, v2, v3, v4, v5):
    """Retorna lista de 36 floats representando uma matriz 6x6 diagonal."""
    m = [0.0] * 36
    m[0]  = v0
    m[7]  = v1
    m[14] = v2
    m[21] = v3
    m[28] = v4
    m[35] = v5
    return m


def main(args=None):
    rclpy.init(args=args)
    node = SensorNoiseInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()