#!/usr/bin/env python3
"""
Armando Mission Controller with Waypoint Execution

Handles mission-specific manipulation tasks for Armando robots.
Implements waypoint navigation for pick and place operations.

Author: Daniele
Date: 2026-01-26
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
import time


class ArmandoMissionController(Node):
    def __init__(self, robot_id):
        super().__init__(f'armando_{robot_id}_mission_controller')
        
        self.robot_id = robot_id
        self.namespace = f'armando_{robot_id}'
        
        # Define waypoints based on robot position
        if robot_id == 1:
            # Colonna DX (station 1) - yaw=0, guarda +X
            self.waypoints = {
                'home': [0.0, 0.0, 0.0, 0.0],
                'hammer': [1.57, -1.6, -0.0, -0.0],     # Posizione bassa verso martello
                'lift': [0.0, 0.0, 0.0, 0.0],          # Alza martello (posizione intermedia sicura)
                'fra2mo': [0.0, -1.6, -0.0, -0.0],    # Posizione bassa verso Fra2mo
            }
        else:  # robot_id == 2
            # Colonna SX (station 2) - yaw=π, guarda -X
            # Specchio: j0 invertito
            self.waypoints = {
                'home': [0.0, 0.0, 0.0, 0.0],
                'fra2mo': [0.0, -1.0, -0.0, -0.0],      # j0 invertito - prende da rover
                'lift': [-1.57, 0.0, 0.0, 0.0],          # j0 invertito - alza
                'cabinet': [-1.57, -1.6, -0.0, -0.0],      # j0 invertito - armadio
            }
        
        # Gripper states
        self.gripper_open = [0.02, 0.02]
        self.gripper_closed = [0.003, 0.003]
        
        # Publishers for joint commands
        self.position_cmd_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self.namespace}/position_controller/commands',
            10
        )
        
        self.gripper_cmd_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self.namespace}/gripper_controller/commands',
            10
        )
        
        # Publisher for mission status
        self.status_pub = self.create_publisher(
            String, 
            f'/{self.namespace}/status', 
            10
        )
        
        # Subscriber for mission commands
        self.cmd_sub = self.create_subscription(
            String,
            f'/{self.namespace}/mission_command',
            self.mission_command_callback,
            10
        )
        
        # Subscriber for joint states (to verify movements)
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/{self.namespace}/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = None
        
        self.get_logger().info(f'🦾 Armando {robot_id} Mission Controller initialized')
        
        # Wait a bit for publishers to be ready
        time.sleep(1.0)
        
        # Move to home position
        self.move_to_waypoint('home')
        self.open_gripper()
        
        # Publish READY status
        time.sleep(0.5)
        self.status_pub.publish(String(data='READY'))
        self.get_logger().info(f'🦾 Armando {robot_id} READY and at HOME position')
    
    def joint_state_callback(self, msg):
        """Store current joint positions"""
        self.current_joint_positions = list(msg.position)
    
    def mission_command_callback(self, msg):
        """Handle mission commands from coordinator"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        if command == 'LOAD_ROVER' and self.robot_id == 1:
            self.execute_loading_task()
        elif command == 'UNLOAD_TO_CABINET' and self.robot_id == 2:
            self.execute_unloading_task()
    
    def move_to_waypoint(self, waypoint_name, duration=2.0):
        """
        Move arm to a predefined waypoint.
        
        Args:
            waypoint_name: Name of waypoint ('home', 'hammer', 'fra2mo', etc.)
            duration: Time to wait for movement completion (seconds)
        """
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'Unknown waypoint: {waypoint_name}')
            return False
        
        target_position = self.waypoints[waypoint_name]
        
        self.get_logger().info(f'Moving to waypoint: {waypoint_name} {target_position}')
        
        # Publish position command
        msg = Float64MultiArray()
        msg.data = target_position
        self.position_cmd_pub.publish(msg)
        
        # Wait for movement to complete
        time.sleep(duration)
        
        self.get_logger().info(f'✅ Reached waypoint: {waypoint_name}')
        return True
    
    def open_gripper(self, duration=1.0):
        """Open gripper"""
        self.get_logger().info('Opening gripper...')
        
        msg = Float64MultiArray()
        msg.data = self.gripper_open
        self.gripper_cmd_pub.publish(msg)
        
        time.sleep(duration)
        self.get_logger().info('✅ Gripper opened')
    
    def close_gripper(self, duration=1.0):
        """Close gripper"""
        self.get_logger().info('Closing gripper...')
        
        msg = Float64MultiArray()
        msg.data = self.gripper_closed
        self.gripper_cmd_pub.publish(msg)
        
        time.sleep(duration)
        self.get_logger().info('✅ Gripper closed')
    
    def execute_loading_task(self):
        """
        Execute loading task for Armando 1.
        Sequence: hammer → lift → fra2mo → home
        """
        self.get_logger().info('='*60)
        self.get_logger().info('Starting LOADING task (Hammer → Fra2mo)')
        self.get_logger().info('='*60)
        
        try:
            # Step 1: Move to hammer position
            self.get_logger().info('Step 1/6: Moving to HAMMER position...')
            self.move_to_waypoint('hammer', duration=3.0)
            
            # Step 2: Close gripper to grasp hammer
            self.get_logger().info('Step 2/6: Grasping HAMMER...')
            self.close_gripper(duration=1.5)
            
            # Step 3: Lift hammer (safe intermediate position)
            self.get_logger().info('Step 3/6: Lifting HAMMER...')
            self.move_to_waypoint('lift', duration=2.5)
            
            # Step 4: Move to Fra2mo position
            self.get_logger().info('Step 4/6: Moving to FRA2MO position...')
            self.move_to_waypoint('fra2mo', duration=3.0)
            
            # Step 5: Open gripper to release hammer
            self.get_logger().info('Step 5/6: Releasing HAMMER on Fra2mo...')
            self.open_gripper(duration=1.5)
            
            # Step 6: Return to home position
            self.get_logger().info('Step 6/6: Returning to HOME position...')
            self.move_to_waypoint('lift', duration=2.0)  # Safe intermediate
            self.move_to_waypoint('home', duration=2.5)
            
            # Publish completion status
            self.status_pub.publish(String(data='TASK_COMPLETE'))
            
            self.get_logger().info('='*60)
            self.get_logger().info('✅ LOADING task COMPLETE')
            self.get_logger().info('='*60)
            
        except Exception as e:
            self.get_logger().error(f'❌ Loading task FAILED: {str(e)}')
            self.status_pub.publish(String(data='TASK_FAILED'))
            # Try to return home safely
            self.open_gripper()
            self.move_to_waypoint('home')
    
    def execute_unloading_task(self):
        """
        Execute unloading task for Armando 2.
        Sequence: fra2mo → lift → cabinet → home
        """
        self.get_logger().info('='*60)
        self.get_logger().info('Starting UNLOADING task (Fra2mo → Cabinet)')
        self.get_logger().info('='*60)
        
        try:
            # Step 1: Move to Fra2mo position
            self.get_logger().info('Step 1/6: Moving to FRA2MO position...')
            self.move_to_waypoint('fra2mo', duration=3.0)
            
            # Step 2: Close gripper to grasp hammer
            self.get_logger().info('Step 2/6: Grasping HAMMER from Fra2mo...')
            self.close_gripper(duration=1.5)
            
            # Step 3: Lift hammer (safe intermediate position)
            self.get_logger().info('Step 3/6: Lifting HAMMER...')
            self.move_to_waypoint('lift', duration=2.5)
            
            # Step 4: Move to cabinet position
            self.get_logger().info('Step 4/6: Moving to CABINET position...')
            self.move_to_waypoint('cabinet', duration=3.0)
            
            # Step 5: Open gripper to release hammer in cabinet
            self.get_logger().info('Step 5/6: Placing HAMMER in CABINET...')
            self.open_gripper(duration=1.5)
            
            # Step 6: Return to home position
            self.get_logger().info('Step 6/6: Returning to HOME position...')
            self.move_to_waypoint('lift', duration=2.0)  # Safe intermediate
            self.move_to_waypoint('home', duration=2.5)
            
            # Publish completion status
            self.status_pub.publish(String(data='TASK_COMPLETE'))
            
            self.get_logger().info('='*60)
            self.get_logger().info('✅ UNLOADING task COMPLETE')
            self.get_logger().info('='*60)
            
        except Exception as e:
            self.get_logger().error(f'❌ Unloading task FAILED: {str(e)}')
            self.status_pub.publish(String(data='TASK_FAILED'))
            # Try to return home safely
            self.open_gripper()
            self.move_to_waypoint('home')


def main():
    import sys
    
    if len(sys.argv) < 2:
        print('Usage: ros2 run package armando_mission_controller.py <robot_id>')
        print('Example: ros2 run ros2_fra2mo armando_mission_controller.py 1')
        return
    
    robot_id = int(sys.argv[1])
    
    if robot_id not in [1, 2]:
        print('Error: robot_id must be 1 or 2')
        return
    
    rclpy.init()
    
    try:
        controller = ArmandoMissionController(robot_id)
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('\n⚠️ Interrupted by user')
    except Exception as e:
        print(f'❌ Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
