#!/usr/bin/env python3
"""
Coordinated Mission Launch File

Launches mission coordinator and all robot controllers for coordinated operation.

Usage:
    ros2 launch ros2_fra2mo waypoint_follow.launch.py

Author: Daniele
Date: 2026-01-26
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    
    # Mission Coordinator (starts immediately)
    mission_coordinator = Node(
        package='final_proj',  # 
        executable='coordinator.py',
        name='mission_coordinator',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )
    
    # Armando 1 Mission Controller (starts after 2 seconds)
    armando1_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='final_proj',  # O 'final_project' se è lì
                executable='armando_controllers.py',
                name='armando_1_mission_controller',
                arguments=['1'],
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Armando 2 Mission Controller (starts after 4 seconds)
    armando2_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='final_proj',  # O 'final_project' se è lì
                executable='armando_controllers.py',
                name='armando_2_mission_controller',
                arguments=['2'],
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Rover Waypoint Navigation Controller (starts after 6 seconds)
    rover_navigation = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros2_fra2mo', 
                executable='navigate.py',
                name='waypoint_navigation',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    return LaunchDescription([
        mission_coordinator,
        armando1_controller,
        armando2_controller,
        rover_navigation
    ])
