import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_symulator_ode')

    config_file    = os.path.join(pkg_share, 'config', 'params.yaml')
    urdf_file      = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_config    = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Optional: allow disabling RViz via launch argument
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz2 visualization'
    )

    simulator_node = Node(
        package='mobile_robot_symulator_ode',
        executable='mobile_robot_simulator_node',
        name='mobile_robot_simulator_node',
        parameters=[config_file],
        output='screen'
    )

    controller_node = Node(
        package='mobile_robot_symulator_ode',
        executable='controller_node',
        name='controller_node',
        parameters=[config_file],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=__import__(
            'launch.conditions', fromlist=['IfCondition']
        ).IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        simulator_node,
        controller_node,
        robot_state_publisher,
        rviz_node,
    ])
