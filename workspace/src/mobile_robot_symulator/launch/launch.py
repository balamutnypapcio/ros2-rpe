import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_symulator')
    
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz') 

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    nodes_to_start = [
        Node(
            package='mobile_robot_symulator',
            executable='controller_node',
            name='controller_node',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='mobile_robot_symulator',
            executable='mobile_robot_simulator_node',
            name='mobile_robot_simulator_node',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ]

    return LaunchDescription(nodes_to_start)