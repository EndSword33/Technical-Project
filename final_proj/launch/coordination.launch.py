#!/usr/bin/env python3
"""
Complete Mission Launch File

Launches all nodes for coordinated mission.

Usage:
    ros2 launch ros2_fra2mo coordination.launch.py

Author: Daniele
Date: 2026-01-26
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    
    # Mission Coordinator (starts first)
    mission_coordinator = Node(
        package='ros2_fra2mo',
        executable='coordinator.py',
        name='mission_coordinator',
        output='screen',
        emulate_tty=True
    )
    
    # Armando 1 Mission Controller
    armando1_controller = Node(
        package='ros2_fra2mo',
        executable='armando_controllers.py',
        name='armando_1_mission_controller',
        arguments=['1'],
        output='screen',
        emulate_tty=True
    )
    
    # Armando 2 Mission Controller
    armando2_controller = Node(
        package='ros2_fra2mo',
        executable='armando_controllers.py',
        name='armando_2_mission_controller',
        arguments=['2'],
        output='screen',
        emulate_tty=True
    )
    
    # Rover Waypoint Navigation (modified)
    rover_navigation = Node(
        package='ros2_fra2mo',
        executable='navigate.py',
        name='waypoint_navigation',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        mission_coordinator,
        armando1_controller,
        armando2_controller,
        rover_navigation
    ])
