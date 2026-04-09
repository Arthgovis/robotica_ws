from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_localization')
    ekf_yaml  = os.path.join(pkg_share, 'config', 'ekf.yaml')

    noise_x_arg    = DeclareLaunchArgument('noise_x_std',    default_value='0.15')
    noise_y_arg    = DeclareLaunchArgument('noise_y_std',    default_value='0.15')
    noise_yaw_arg  = DeclareLaunchArgument('noise_yaw_std',  default_value='0.08')
    noise_vx_arg   = DeclareLaunchArgument('noise_vx_std',   default_value='0.03')
    noise_vyaw_arg = DeclareLaunchArgument('noise_vyaw_std', default_value='0.2')

    noise_injector_node = Node(
        package='rm_localization',
        executable='sensor_noise_injector',
        name='sensor_noise_injector',
        output='screen',
        parameters=[{
            'noise_x_std':    LaunchConfiguration('noise_x_std'),
            'noise_y_std':    LaunchConfiguration('noise_y_std'),
            'noise_yaw_std':  LaunchConfiguration('noise_yaw_std'),
            'noise_vx_std':   LaunchConfiguration('noise_vx_std'),
            'noise_vyaw_std': LaunchConfiguration('noise_vyaw_std'),
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': True}],
    )

    return LaunchDescription([
        noise_x_arg, noise_y_arg, noise_yaw_arg,
        noise_vx_arg, noise_vyaw_arg,
        noise_injector_node,
        ekf_node,
    ])