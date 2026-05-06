from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('prestop'),
                'config',
                'prestop.yaml',
            ]),
            description='Path to prestop parameters.',
        ),
        Node(
            package='prestop',
            executable='prestop_node',
            name='prestop_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
