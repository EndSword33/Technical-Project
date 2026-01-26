#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for waypoint navigation.
    """
    
    # Waypoint navigation node
    waypoint_navigation_node = Node(
        package='ros2_fra2mo',  # Sostituisci con il nome del tuo package
        executable='navigate.py',
        name='navigate',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': True}
        ]
    )
    
    return LaunchDescription([
        waypoint_navigation_node
    ])
