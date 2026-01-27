#!/usr/bin/env python3
"""
Mission Coordinator Node

Central coordinator for Fra2mo rover and Armando robots.
Implements workflow using state machine.

Workflow:
1. Rover waits at initial position
2. Armando 1 moves from hammer to rover and returns to initial pose
3. Rover navigates to waypoint 1
4. Rover navigates to waypoint 2 and waits
5. Armando 2 picks hammer and stores in cabinet, returns to initial pose
6. Rover continues to remaining waypoints

Author: Franco
Date: 2026-01-26
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from enum import Enum
import time


class MissionState(Enum):
    """Mission states for the workflow"""
    INIT = 0
    WAITING_ROVER_READY = 1
    ARMANDO1_LOADING = 2
    WAITING_ARMANDO1_DONE = 3
    ROVER_TO_WP1 = 4
    ROVER_TO_WP2 = 5
    WAITING_ROVER_AT_WP2 = 6
    ARMANDO2_UNLOADING = 7
    WAITING_ARMANDO2_DONE = 8
    ROVER_TO_WP3 = 9
    ROVER_TO_HOME = 10
    MISSION_COMPLETE = 11
    MISSION_FAILED = 12


class MissionCoordinator(Node):
    """
    Central coordinator node for multi-robot mission.
    """
    
    def __init__(self):
        super().__init__('mission_coordinator')
        
        # State machine
        self.current_state = MissionState.INIT
        
        # Robot status flags
        self.rover_ready = False
        self.rover_at_waypoint = False
        self.armando1_ready = False
        self.armando1_done = False
        self.armando2_ready = False
        self.armando2_done = False
        
        # Publishers - Commands to robots
        self.rover_cmd_pub = self.create_publisher(
            String, 
            '/rover/mission_command', 
            10
        )
        
        self.armando1_cmd_pub = self.create_publisher(
            String, 
            '/armando_1/mission_command', 
            10
        )
        
        self.armando2_cmd_pub = self.create_publisher(
            String, 
            '/armando_2/mission_command', 
            10
        )
        
        # Subscribers - Status from robots
        self.rover_status_sub = self.create_subscription(
            String,
            '/rover/status',
            self.rover_status_callback,
            10
        )
        
        self.armando1_status_sub = self.create_subscription(
            String,
            '/armando_1/status',
            self.armando1_status_callback,
            10
        )
        
        self.armando2_status_sub = self.create_subscription(
            String,
            '/armando_2/status',
            self.armando2_status_callback,
            10
        )
        
        # Timer for state machine execution (10 Hz)
        self.timer = self.create_timer(0.1, self.state_machine_update)
        
        self.get_logger().info('🎯 Mission Coordinator initialized')
        self.get_logger().info('Waiting for robots to be ready...')
    
    # ============================================
    # CALLBACKS - Robot Status Updates
    # ============================================
    
    def rover_status_callback(self, msg):
        """Process rover status updates"""
        status = msg.data
        
        if status == 'READY':
            self.rover_ready = True
            self.get_logger().info('🚙 Rover is READY')
            
        elif status == 'ARRIVED_WP1':
            self.rover_at_waypoint = True
            self.get_logger().info('🚙 Rover arrived at Waypoint 1')
            
        elif status == 'ARRIVED_WP2':
            self.rover_at_waypoint = True
            self.get_logger().info('🚙 Rover arrived at Waypoint 2')
            
        elif status == 'ARRIVED_HOME':
            self.rover_at_waypoint = True
            self.get_logger().info('🚙 Rover arrived HOME')
            
        elif status == 'NAVIGATION_FAILED':
            self.get_logger().error('🚙 Rover navigation FAILED')
            self.current_state = MissionState.MISSION_FAILED
    
    def armando1_status_callback(self, msg):
        """Process Armando 1 status updates"""
        status = msg.data
        
        if status == 'READY':
            self.armando1_ready = True
            self.get_logger().info('🦾 Armando 1 is READY')
            
        elif status == 'TASK_COMPLETE':
            self.armando1_done = True
            self.get_logger().info('🦾 Armando 1 task COMPLETE')
            
        elif status == 'TASK_FAILED':
            self.get_logger().error('🦾 Armando 1 task FAILED')
            self.current_state = MissionState.MISSION_FAILED
    
    def armando2_status_callback(self, msg):
        """Process Armando 2 status updates"""
        status = msg.data
        
        if status == 'READY':
            self.armando2_ready = True
            self.get_logger().info('🦾 Armando 2 is READY')
            
        elif status == 'TASK_COMPLETE':
            self.armando2_done = True
            self.get_logger().info('🦾 Armando 2 task COMPLETE')
            
        elif status == 'TASK_FAILED':
            self.get_logger().error('🦾 Armando 2 task FAILED')
            self.current_state = MissionState.MISSION_FAILED
    
    # ============================================
    # STATE MACHINE
    # ============================================
    
    def state_machine_update(self):
        """State machine execution - called at 10 Hz"""
        
        if self.current_state == MissionState.INIT:
            self.get_logger().info('📍 State: INIT')
            self.current_state = MissionState.WAITING_ROVER_READY
        
        elif self.current_state == MissionState.WAITING_ROVER_READY:
            # Wait for rover and armando1 to be ready
            if self.rover_ready and self.armando1_ready:
                self.get_logger().info('✅ All robots ready. Starting mission...')
                self.current_state = MissionState.ARMANDO1_LOADING
        
        elif self.current_state == MissionState.ARMANDO1_LOADING:
            self.get_logger().info('📍 State: ARMANDO1_LOADING')
            self.get_logger().info('🦾 Commanding Armando 1 to load rover...')
            
            # Command Armando 1 to execute loading task
            self.armando1_cmd_pub.publish(String(data='LOAD_ROVER'))
            
            self.current_state = MissionState.WAITING_ARMANDO1_DONE
        
        elif self.current_state == MissionState.WAITING_ARMANDO1_DONE:
            # Wait for Armando 1 to complete loading
            if self.armando1_done:
                self.get_logger().info('✅ Armando 1 loading complete')
                self.current_state = MissionState.ROVER_TO_WP1
        
        elif self.current_state == MissionState.ROVER_TO_WP1:
            self.get_logger().info('📍 State: ROVER_TO_WP1')
            self.get_logger().info('🚙 Commanding rover to Waypoint 1...')
            
            # Command rover to navigate to WP1
            self.rover_cmd_pub.publish(String(data='GOTO_WP1'))
            
            self.rover_at_waypoint = False
            self.current_state = MissionState.ROVER_TO_WP2
        
        elif self.current_state == MissionState.ROVER_TO_WP2:
            # Wait for rover to reach WP1, then go to WP2
            if self.rover_at_waypoint:
                self.get_logger().info('📍 State: ROVER_TO_WP2')
                self.get_logger().info('🚙 Commanding rover to Waypoint 2...')
                
                self.rover_cmd_pub.publish(String(data='GOTO_WP2'))
                
                self.rover_at_waypoint = False
                self.current_state = MissionState.WAITING_ROVER_AT_WP2
        
        elif self.current_state == MissionState.WAITING_ROVER_AT_WP2:
            # Wait for rover to reach WP2
            if self.rover_at_waypoint:
                self.get_logger().info('✅ Rover at Waypoint 2')
                self.current_state = MissionState.ARMANDO2_UNLOADING
        
        elif self.current_state == MissionState.ARMANDO2_UNLOADING:
            self.get_logger().info('📍 State: ARMANDO2_UNLOADING')
            self.get_logger().info('🦾 Commanding Armando 2 to unload...')
            
            # Command Armando 2 to execute unloading task
            self.armando2_cmd_pub.publish(String(data='UNLOAD_TO_CABINET'))
            
            self.current_state = MissionState.WAITING_ARMANDO2_DONE
        
        elif self.current_state == MissionState.WAITING_ARMANDO2_DONE:
            # Wait for Armando 2 to complete unloading
            if self.armando2_done:
                self.get_logger().info('✅ Armando 2 unloading complete')
                self.current_state = MissionState.ROVER_TO_WP3
        
        elif self.current_state == MissionState.ROVER_TO_WP3:
            self.get_logger().info('📍 State: ROVER_TO_WP3')
            self.get_logger().info('🚙 Commanding rover to Waypoint 3...')
            
            # Command rover to continue to WP3
            self.rover_cmd_pub.publish(String(data='CONTINUE_MISSION'))
            
            self.rover_at_waypoint = False
            self.current_state = MissionState.ROVER_TO_HOME
        
        elif self.current_state == MissionState.ROVER_TO_HOME:
            # Wait for rover to complete remaining waypoints
            if self.rover_at_waypoint:
                self.get_logger().info('✅ Rover returned HOME')
                self.current_state = MissionState.MISSION_COMPLETE
        
        elif self.current_state == MissionState.MISSION_COMPLETE:
            self.get_logger().info(
                '\n' + '='*60 + '\n' +
                '🎉 MISSION COMPLETE! 🎉\n' +
                '='*60 + '\n'
            )
            # Stop timer
            self.timer.cancel()
        
        elif self.current_state == MissionState.MISSION_FAILED:
            self.get_logger().error(
                '\n' + '='*60 + '\n' +
                '❌ MISSION FAILED ❌\n' +
                '='*60 + '\n'
            )
            # Stop timer
            self.timer.cancel()


def main():
    rclpy.init()
    
    try:
        coordinator = MissionCoordinator()
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
