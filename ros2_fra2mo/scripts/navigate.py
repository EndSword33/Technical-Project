#!/usr/bin/env python3
"""
Coordinated Waypoint Navigation Node

This node navigates Fra2mo through waypoints with coordinator integration.

Author: Franco
Date: 2026-01-26
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import sys
import time


# ============================================
# WAYPOINT CONFIGURATION
# ============================================

WAYPOINTS = [
    # Primo waypoint
    {'name': 'Waypoint_1', 'x': 1.45, 'y': -0.111, 'yaw': 0.0},
    
    # Secondo waypoint
    {'name': 'Waypoint_2', 'x': 1.6, 'y': -2.33, 'yaw': 0.0},
    
    # Ritorno al primo waypoint con yaw diverso
    {'name': 'Waypoint_3', 'x': 1.45, 'y': -0.111, 'yaw': 3.14},
    
    # Ritorno alla posizione iniziale
    {'name': 'Home', 'x': -0.02, 'y': -0.02, 'yaw': 3.14},
]


# ============================================
# HELPER FUNCTIONS
# ============================================

def yaw_to_quaternion(yaw):
    """
    Convert yaw angle to quaternion.
    
    Args:
        yaw: Rotation angle in radians around Z axis
        
    Returns:
        List [qx, qy, qz, qw]
    """
    return [
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0)
    ]


def create_goal_pose(navigator, x, y, yaw):
    """
    Create a PoseStamped goal pose in map frame.
    
    Args:
        navigator: BasicNavigator instance
        x: X coordinate in map frame
        y: Y coordinate in map frame
        yaw: Orientation in radians
        
    Returns:
        PoseStamped message
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    
    q = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    
    return pose


# ============================================
# COORDINATED WAYPOINT NAVIGATION NODE
# ============================================

class CoordinatedWaypointNavigation(Node):
    """
    Node for coordinated waypoint navigation with mission controller.
    """
    
    def __init__(self):
        super().__init__('waypoint_navigation')
        
        # Initialize navigator
        self.navigator = BasicNavigator()
        
        # Publisher for rover status
        self.status_pub = self.create_publisher(String, '/rover/status', 10)
        
        # Subscriber for mission commands
        self.cmd_sub = self.create_subscription(
            String,
            '/rover/mission_command',
            self.mission_command_callback,
            10
        )
        
        # Mission state
        self.current_waypoint_index = 0
        self.mission_active = False
        self.waiting_for_command = True
        
        self.get_logger().info('🚙 Coordinated Waypoint Navigation initialized')
        
        # Wait for Nav2
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('🟢 Nav2 is active!')
        
        # Publish READY status
        time.sleep(1.0)
        self.status_pub.publish(String(data='READY'))
        self.get_logger().info('🚙 Rover READY and waiting for coordinator commands...')
    
    def mission_command_callback(self, msg):
        """Handle commands from mission coordinator"""
        command = msg.data
        self.get_logger().info(f'📨 Received command: {command}')
        
        if command == 'GOTO_WP1':
            self.get_logger().info('Starting navigation to Waypoint 1...')
            self.current_waypoint_index = 0
            self.navigate_to_waypoint(WAYPOINTS[0])
            
        elif command == 'GOTO_WP2':
            self.get_logger().info('Starting navigation to Waypoint 2...')
            self.current_waypoint_index = 1
            self.navigate_to_waypoint(WAYPOINTS[1])
            
        elif command == 'CONTINUE_MISSION':
            self.get_logger().info('Continuing mission to remaining waypoints...')
            self.current_waypoint_index = 2
            self.navigate_remaining_waypoints()
    
    def navigate_to_waypoint(self, waypoint):
        """
        Navigate to a single waypoint.
        
        Args:
            waypoint: Dictionary with 'name', 'x', 'y', 'yaw'
        """
        self.get_logger().info(
            f'\n{"="*60}\n'
            f'Navigating to {waypoint["name"]}\n'
            f'Target: x={waypoint["x"]:.3f}, y={waypoint["y"]:.3f}, '
            f'yaw={waypoint["yaw"]:.2f} rad ({math.degrees(waypoint["yaw"]):.1f}°)\n'
            f'{"="*60}'
        )
        
        # Create goal pose
        goal_pose = create_goal_pose(
            self.navigator,
            waypoint['x'],
            waypoint['y'],
            waypoint['yaw']
        )
        
        # Start navigation
        nav_start = self.navigator.get_clock().now()
        self.navigator.goToPose(goal_pose)
        
        # Monitor navigation progress
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            
            # Log progress every 10 iterations
            if feedback and i % 10 == 0:
                current_pose = feedback.current_pose.pose
                distance_remaining = math.sqrt(
                    (waypoint['x'] - current_pose.position.x)**2 +
                    (waypoint['y'] - current_pose.position.y)**2
                )
                
                self.get_logger().info(
                    f'🚙 Distance remaining: {distance_remaining:.2f}m'
                )
            
            # Safety timeout (3 minutes per waypoint)
            now = self.navigator.get_clock().now()
            if now - nav_start > Duration(seconds=180):
                self.get_logger().warn(
                    f'Navigation timeout (3 min) for {waypoint["name"]}. Canceling...'
                )
                self.navigator.cancelTask()
                break
        
        # Check result
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'✅ Successfully reached {waypoint["name"]}!\n')
            
            # Publish appropriate status
            if waypoint['name'] == 'Waypoint_1':
                self.status_pub.publish(String(data='ARRIVED_WP1'))
            elif waypoint['name'] == 'Waypoint_2':
                self.status_pub.publish(String(data='ARRIVED_WP2'))
            elif waypoint['name'] == 'Home':
                self.status_pub.publish(String(data='ARRIVED_HOME'))
            
            return True
            
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f'❌ Navigation to {waypoint["name"]} was canceled!')
            self.status_pub.publish(String(data='NAVIGATION_FAILED'))
            return False
            
        elif result == TaskResult.FAILED:
            self.get_logger().error(f'❌ Navigation to {waypoint["name"]} failed!')
            self.status_pub.publish(String(data='NAVIGATION_FAILED'))
            return False
            
        else:
            self.get_logger().error(
                f'❌ Navigation to {waypoint["name"]} returned invalid status: {result}'
            )
            self.status_pub.publish(String(data='NAVIGATION_FAILED'))
            return False
    
    def navigate_remaining_waypoints(self):
        """Navigate through remaining waypoints (WP3 and Home)"""
        for idx in range(self.current_waypoint_index, len(WAYPOINTS)):
            waypoint = WAYPOINTS[idx]
            success = self.navigate_to_waypoint(waypoint)
            
            if not success:
                self.get_logger().error(f'Failed to reach {waypoint["name"]}')
                return False
            
            # Small pause between waypoints
            if idx < len(WAYPOINTS) - 1:
                self.get_logger().info('Pausing 2 seconds before next waypoint...\n')
                time.sleep(2.0)
        
        self.get_logger().info(
            f'\n{"🎉"*20}\n'
            f'Rover mission complete! All waypoints reached.\n'
            f'{"🎉"*20}\n'
        )
        return True


def main():
    rclpy.init()
    
    node = None
    
    try:
        node = CoordinatedWaypointNavigation()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n⚠️ Interrupted by user')
    except Exception as e:
        print(f'❌ Error: {e}')
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except:
                pass
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
