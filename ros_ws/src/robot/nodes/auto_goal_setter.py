#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import random
import math
import numpy as np


class AutoGoalSetter(Node):
    def __init__(self):
        super().__init__('auto_goal_setter')
        
        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribe to costmap for wall detection
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )
        
        # Subscribe to behavior tree status for recovery detection
        self.bt_status_sub = self.create_subscription(
            String,
            '/behavior_tree_log',
            self.bt_status_callback,
            10
        )
        
        self.costmap = None
        self.in_recovery = False
        self.goal_handle = None
        self.min_distance_from_wall = 0.3 # how far should be a goal from wall [m]
        self.min_distance_from_map_edge = 2.5  # how far should be a goal from map edges [m]
        self.last_robot_pose = None
        self.last_progress_time = self.get_clock().now()
        self.stuck_timeout = 5.0  # time to get robot to move [s]
        self.min_progress_distance = 0.15  # what distance make decide, that robot move - not stucked [m]

        # Subscribe to get actual position of robot
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.robot_pose_callback,
            10
        )
        
        # Define forbidden regions
        # Each region is defined as a dictionary with conditions
        self.forbidden_regions = [
            # Willow garage limits
            {'x_min': 30, 'x_max': float('inf'), 'y_min': float('-inf'), 'y_max': float('inf')},  # x >= 30
            {'x_min': float('-inf'), 'x_max': -20, 'y_min': float('-inf'), 'y_max': float('inf')},  # x <= -20
            {'x_min': float('-inf'), 'x_max': float('inf'), 'y_min': float('-inf'), 'y_max': -20},  # y >-19
            {'x_min': -20.0, 'x_max': -14.0, 'y_min': -20.0, 'y_max': -1.0},
            {'x_min': -20.0, 'x_max': -14.0, 'y_min': 18.0, 'y_max': 25.0},                    
        ]
        
        # Wait for action server
        self.get_logger().info('Waiting for navigation action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Navigation action server available!')
        
        # Start sending goals
        self.timer = self.create_timer(2.0, self.check_and_send_goal)
        self.goal_active = False
        
    def costmap_callback(self, msg):
        """Store the latest costmap data"""
        self.costmap = msg
        
    def bt_status_callback(self, msg):
        """Detect recovery behavior from behavior tree logs"""
        # Common recovery behavior indicators in Nav2
        recovery_keywords = ['recovery', 'spin', 'backup', 'wait']
        if any(keyword in msg.data.lower() for keyword in recovery_keywords):
            if not self.in_recovery:
                self.get_logger().warn('Recovery behavior detected!')
                self.in_recovery = True
                self.cancel_current_goal()
        
    def is_in_forbidden_region(self, x, y):
        """Check if position is in any forbidden region"""
        for region in self.forbidden_regions:
            if (region['x_min'] <= x <= region['x_max'] and 
                region['y_min'] <= y <= region['y_max']):
                return True
        return False
    
    def is_valid_position(self, x, y):
        """Check if position is valid (away from walls, map edges, and forbidden regions) using costmap"""
        if self.costmap is None:
            return True
        
        # Check if in forbidden region
        if self.is_in_forbidden_region(x, y):
            return False
        
        # Convert world coordinates to map coordinates
        map_x = int((x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
        map_y = int((y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
        
        # Calculate cells for map edge distance
        edge_cells = int(self.min_distance_from_map_edge / self.costmap.info.resolution)
        
        # Check if too close to map edges
        if map_x < edge_cells or map_x >= (self.costmap.info.width - edge_cells) or \
           map_y < edge_cells or map_y >= (self.costmap.info.height - edge_cells):
            return False
        
        # Calculate cells for minimum distance from wall
        cells_distance = int(self.min_distance_from_wall / self.costmap.info.resolution)
        
        # Check area around the point for obstacles
        for dx in range(-cells_distance, cells_distance + 1):
            for dy in range(-cells_distance, cells_distance + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                
                if check_x < 0 or check_x >= self.costmap.info.width or \
                   check_y < 0 or check_y >= self.costmap.info.height:
                    continue
                
                idx = check_y * self.costmap.info.width + check_x
                
                # Check if occupied (value > 50 typically means obstacle)
                if idx < len(self.costmap.data) and self.costmap.data[idx] > 50:
                    return False
        
        return True
    
    def generate_random_goal(self):
        """Generate a random goal position away from walls and map edges"""
        max_attempts = 50
        
        if self.costmap is None:
            # If no costmap, generate in a safe range
            x = random.uniform(-5.0, 5.0)
            y = random.uniform(-5.0, 5.0)
        else:
            # Calculate safe boundaries (in world coordinates)
            edge_distance_world = self.min_distance_from_map_edge
            
            x_min = self.costmap.info.origin.position.x + edge_distance_world
            x_max = self.costmap.info.origin.position.x + \
                    (self.costmap.info.width * self.costmap.info.resolution) - edge_distance_world
            
            y_min = self.costmap.info.origin.position.y + edge_distance_world
            y_max = self.costmap.info.origin.position.y + \
                    (self.costmap.info.height * self.costmap.info.resolution) - edge_distance_world
            
            # Try to find a valid position
            for attempt in range(max_attempts):
                # Generate random position within safe boundaries
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)
                
                if self.is_valid_position(x, y):
                    self.get_logger().info(f'Valid position found on attempt {attempt + 1}')
                    break
            else:
                self.get_logger().warn('Could not find valid position after all attempts, using constrained random')
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)
        
        # Random orientation
        theta = random.uniform(-math.pi, math.pi)
        
        return x, y, theta
    
    def send_goal(self):
        """Send a new navigation goal"""
        x, y, theta = self.generate_random_goal()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_active = True
        self.in_recovery = False
        
    def goal_response_callback(self, future):
        """Handle goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.goal_active = False
            return
        
        self.get_logger().info('Goal accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def feedback_callback(self, feedback_msg):
        """Process navigation feedback"""
        # You can monitor progress here if needed
        pass
        
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached successfully!')
        elif status == 5:  # CANCELED
            self.get_logger().warn('Goal was canceled')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
        
        self.goal_active = False
        
    def cancel_current_goal(self):
        """Cancel the current navigation goal"""
        if self.goal_handle is not None and self.goal_active:
            self.get_logger().info('Canceling current goal due to recovery')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: setattr(self, 'goal_active', False))
            
    def check_and_send_goal(self):
        """Periodically check if we need to send a new goal"""
        if not self.goal_active or self.in_recovery:
            if self.in_recovery:
                self.get_logger().info('Sending new goal after recovery')
            self.send_goal()

    def robot_pose_callback(self, msg):
        if not self.goal_active:
            return

        current_time = self.get_clock().now()
        curr_x = msg.pose.position.x
        curr_y = msg.pose.position.y

        if self.last_robot_pose is not None:
            dist = math.sqrt((curr_x - self.last_robot_pose[0])**2 + 
                            (curr_y - self.last_robot_pose[1])**2)
            
            if dist > self.min_progress_distance:
                self.last_progress_time = current_time
                self.last_robot_pose = (curr_x, curr_y)
                
            elif (current_time - self.last_progress_time).nanoseconds / 1e9 > self.stuck_timeout:
                self.get_logger().error('No progress - robot is blocked. New goal')
                self.in_recovery = True # To wyzwoli send_goal w Twoim timerze
                self.cancel_current_goal()
        else:
            self.last_robot_pose = (curr_x, curr_y)
            self.last_progress_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = AutoGoalSetter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()