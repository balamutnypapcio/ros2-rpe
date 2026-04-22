import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rob_mon_sys')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    with open(config_file, 'r') as f:
        params = yaml.safe_load(f)
    sensors_to_launch = params['StateMonitor']['ros__parameters']['monitored_sensors']
    sensor_publish_period = params['StateMonitor']['ros__parameters']['sensor_publish_period']
    nodes_to_start = []
    nodes_to_start.append(
        Node(
            package='rob_mon_sys',
            executable='StateMonitor',
            name='StateMonitor',
            parameters=[config_file]
        )
    )

    for s_name in sensors_to_launch:
        nodes_to_start.append(
            Node(
                package='rob_mon_sys',
                executable='sensor',
                name=f'sensor_{s_name}',
                parameters=[{
                    'sensor_name': s_name, 
                    'sensor_publish_period': sensor_publish_period
                }]
            )
        )

    nodes_to_start.append(
        Node(
            package='rob_mon_sys',
            executable='ErrorHandler',
            name='ErrorHandler',
            parameters=[config_file]
        )
    )

    return LaunchDescription(nodes_to_start)