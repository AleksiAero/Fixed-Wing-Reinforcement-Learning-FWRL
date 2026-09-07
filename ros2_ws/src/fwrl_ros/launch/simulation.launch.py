from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('python_controller', default_value='true'),
        Node(package='fwrl_ros', executable='simulator', parameters=[{'world': LaunchConfiguration('world')}], output='screen'),
        Node(package='fwrl_ros', executable='controller', condition=IfCondition(LaunchConfiguration('python_controller')), output='screen'),
    ])
