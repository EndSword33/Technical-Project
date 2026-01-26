#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import sys


# ============================================
# WAYPOINT CONFIGURATION
# ============================================

WAYPOINTS = [
    # Primo waypoint
    {'name': 'Waypoint 1', 'x': 1.45, 'y': -0.111, 'yaw': -1.57},
    
    # Secondo waypoint
    {'name': 'Waypoint 2', 'x': 1.27, 'y': -2.42, 'yaw': -1.57},
    
    # Ritorno al primo waypoint con yaw diverso
    {'name': 'Waypoint 1 (return)', 'x': 1.45, 'y': -0.111, 'yaw': 3.14},
    
    # Ritorno alla posizione iniziale
    {'name': 'Initial Position (return)', 'x': -0.127, 'y': -0.04, 'yaw': 3.14},
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
        0.0,                    # qx
        0.0,                    # qy
        math.sin(yaw / 2.0),   # qz
        math.cos(yaw / 2.0)    # qw
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
    
    # Position
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    
    # Orientation (from yaw)
    q = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    
    return pose


# ============================================
# WAYPOINT NAVIGATION NODE
# ============================================

class WaypointNavigationNode(Node):
    """
    Node for autonomous navigation through multiple waypoints.
    """
    
    def __init__(self):
        super().__init__('waypoint_navigation')
        
        # Initialize navigator
        self.navigator = BasicNavigator()
        
        # Publisher for navigation status
        self.navigation_status_pub = self.create_publisher(
            String, 
            '/navigation_status', 
            10
        )
        
        self.get_logger().info('Waypoint Navigation Node initialized')
        self.get_logger().info(f'Total waypoints to navigate: {len(WAYPOINTS)}')
    
    def navigate_to_waypoint(self, waypoint, waypoint_index):
        """
        Navigate to a single waypoint.
        
        Args:
            waypoint: Dictionary with 'name', 'x', 'y', 'yaw'
            waypoint_index: Index of current waypoint (for logging)
            
        Returns:
            True if navigation succeeded, False otherwise
        """
        self.get_logger().info(
            f'\n{"="*60}\n'
            f'Navigating to {waypoint["name"]} ({waypoint_index + 1}/{len(WAYPOINTS)})\n'
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
        
        # Publish navigation status: NAVIGATING
        self.navigation_status_pub.publish(
            String(data=f'NAVIGATING_TO_{waypoint["name"].upper().replace(" ", "_")}')
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
            self.get_logger().info(
                f'✅ Successfully reached {waypoint["name"]}!\n'
            )
            self.navigation_status_pub.publish(
                String(data=f'ARRIVED_AT_{waypoint["name"].upper().replace(" ", "_")}')
            )
            return True
            
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f'❌ Navigation to {waypoint["name"]} was canceled!')
            self.navigation_status_pub.publish(String(data='CANCELED'))
            return False
            
        elif result == TaskResult.FAILED:
            self.get_logger().error(f'❌ Navigation to {waypoint["name"]} failed!')
            self.navigation_status_pub.publish(String(data='FAILED'))
            return False
            
        else:
            self.get_logger().error(
                f'❌ Navigation to {waypoint["name"]} returned invalid status: {result}'
            )
            self.navigation_status_pub.publish(String(data='UNKNOWN_ERROR'))
            return False
    
    def navigate_through_waypoints(self):
        """
        Navigate through all waypoints sequentially.
        
        Returns:
            True if all waypoints were reached successfully, False otherwise
        """
        # Wait for Nav2 to be active
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('🟢 Nav2 is active!')
        
        # Navigate through each waypoint
        for idx, waypoint in enumerate(WAYPOINTS):
            success = self.navigate_to_waypoint(waypoint, idx)
            
            if not success:
                self.get_logger().error(
                    f'Mission aborted at waypoint {idx + 1}/{len(WAYPOINTS)}'
                )
                return False
            
            # Small pause between waypoints
            if idx < len(WAYPOINTS) - 1:
                self.get_logger().info('Pausing 2 seconds before next waypoint...\n')
                rclpy.spin_once(self, timeout_sec=2.0)
        
        # Mission complete
        self.get_logger().info(
            f'\n{"🎉"*20}\n'
            f'MISSION COMPLETE! All {len(WAYPOINTS)} waypoints reached successfully!\n'
            f'{"🎉"*20}\n'
        )
        self.navigation_status_pub.publish(String(data='MISSION_COMPLETE'))
        
        return True


# ============================================
# MAIN
# ============================================

def main():
    # Initialize ROS2
    rclpy.init()
    
    success = False
    node = None
    
    try:
        # Create node
        node = WaypointNavigationNode()
        
        # Execute waypoint navigation
        success = node.navigate_through_waypoints()
        
        # Keep node alive briefly to ensure final messages are sent
        if success:
            node.get_logger().info('Node will shutdown in 2 seconds...')
            rclpy.spin_once(node, timeout_sec=2.0)
        
    except KeyboardInterrupt:
        print('\n⚠️ Interrupted by user')
    except Exception as e:
        print(f'❌ Error: {e}')
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # Cleanup
        if node is not None:
            try:
                node.destroy_node()
            except:
                pass
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
