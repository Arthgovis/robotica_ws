"""
slam.launch.py
--------------
Lanca o SLAM Toolbox via online_async_launch.py (lifecycle node).

Pre-requisito:
  Simulacao rodando: ros2 launch rm_description robot_simulation.launch.py

Uso:
  ros2 launch rm_localization slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_localization')
    slam_yaml = os.path.join(pkg_share, 'config', 'slam_config.yaml')

    slam_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'slam_params_file': slam_yaml,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        slam,
    ])