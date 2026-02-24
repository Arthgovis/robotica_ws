import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Caminho para o arquivo XACRO
    xacro_file = os.path.join(
        get_package_share_directory('rm_description'),
        'urdf',
        # 'robot_arm.xacro'
        'differential_robot.xacro'
    )

    # Processar XACRO para URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    rm_description = doc.toxml()

    # Parâmetros
    params = {
        'robot_description': rm_description,
        'use_sim_time': False
    }

    # Nó robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Nó joint_state_publisher_gui
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Nó RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/arthur/robotica_ws/src/rm_description/rviz/robo_base_2juntas.rviz']
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])