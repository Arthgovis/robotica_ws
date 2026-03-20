from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_description'),
                'launch',
                'robot_simulation.py'
            )
        )
    )

    patrol_node = Node(
        package='patrol_bot',
        executable='patrol_node',
        name='Patrol_bot_node',
        output='screen',
    )

    rotate_node = Node(
        package='patrol_bot',
        executable='rotate_node',
        name='patrol_rotate_server',
        output='screen',
    )

    return LaunchDescription([
        other_launch,
        rotate_node,
        patrol_node,
    ])