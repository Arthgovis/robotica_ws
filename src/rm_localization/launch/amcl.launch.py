"""
localization.launch.py

Lanca o map_server e o AMCL para localizacao com mapa pre-construido.

Uso:
  ros2 launch rm_localization localization.launch.py \
       map:=/caminho/para/casa_map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_localization')
    amcl_yaml = os.path.join(pkg_share, 'config', 'amcl.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Caminho completo para o arquivo .yaml do mapa',
    )

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': LaunchConfiguration('map'),
        }],
        namespace='',
    )

    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml, {'use_sim_time': True}],
        namespace='',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        map_arg,
        map_server,
        amcl,
        lifecycle_manager,
    ])