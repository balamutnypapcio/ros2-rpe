import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_symulator')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    with open(config_file, 'r') as f:
        params = yaml.safe_load(f)
    linear_vel = params['MobileRobotSimulator']['ros__parameters']['linear_vel']
    angular_vel = params['MobileRobotSimulator']['ros__parameters']['angular_vel']
    nodes_to_start = []
    nodes_to_start.append(
        Node(
            package='mobile_robot_symulator',
            executable='controller_node',
            parameters=[config_file]
        )
    )

    return LaunchDescription(nodes_to_start)