#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import math
import time
import heapq
import numpy as np
from typing import List, Tuple, Optional

class BoundedAStarNavigation(Node):
    def __init__(self):
        super().__init__('bounded_astar_navigation')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/dynamic_map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/area_markers', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/simple_nav_command', self.command_callback, 10)
        
        # Define bounded area (rectangular boundary)
        self.boundary_min_x = -3.0
        self.boundary_max_x = 3.0
        self.boundary_min_y = -3.0
        self.boundary_max_y = 3.0
        
        # Fixed waypoints inside the bounded area
        self.waypoints = [
            (-2.0, -2.0),  # Bottom-left
            (2.0, -2.0),   # Bottom-right  
            (2.0, 2.0),    # Top-right
            (-2.0, 2.0),   # Top-left
        ]
        self.current_waypoint_index = 0
        self.waypoint_reached_tolerance = 0.4
        
        # Grid map parameters
        self.map_width = 120   # 12m at 0.1m resolution (covers bounded area + buffer)
        self.map_height = 120
        self.map_resolution = 0.1
        self.map_origin_x = -6.0  # Centered on bounded area
        self.map_origin_y = -6.0
        
        # Initialize occupancy grid
        self.occupancy_grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.dynamic_obstacles = set()
        
        # Robot state
        self.current_pose = None
        self.current_scan = None
        self.target_x = self.waypoints[0][0]
        self.target_y = self.waypoints[0][1]
        self.has_goal = True
        
        # Path planning
        self.current_path = []
        self.current_path_index = 0
        self.path_following = False
        self.planning_active = False
        
        # Control parameters
        self.linear_speed = 0.25
        self.angular_speed = 0.6
        self.goal_tolerance = 0.3
        self.path_point_tolerance = 0.15
        self.obstacle_inflation_radius = 3
        self.obstacle_threshold = 1.5
        
        # A* parameters
        self.max_planning_iterations = 8000
        
        # Control flags
        self.moving = True
        self.autonomous_mode = True
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.map_update_timer = self.create_timer(0.3, self.update_occupancy_grid)
        self.status_timer = self.create_timer(3.0, self.print_status)
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        self.replan_timer = self.create_timer(2.0, self.check_replanning)
        
        # Initialize boundary obstacles in grid
        self.setup_boundary_obstacles()
        
        self.get_logger().info('Bounded A* Navigation Controller started')
        self.get_logger().info(f'Boundary: ({self.boundary_min_x},{self.boundary_min_y}) to ({self.boundary_max_x},{self.boundary_max_y})')
        self.get_logger().info(f'Waypoints: {self.waypoints}')
        self.get_logger().info(f'First target: ({self.target_x}, {self.target_y})')
        
        # Start planning after initialization
        self.create_timer(2.0, self.initial_planning)

    # Add these improvements to your A* navigation controller

    def is_cell_free(self, grid_x: int, grid_y: int, safety_margin: int = 1) -> bool:
        """Check if cell and surrounding area is free"""
        if not self.is_valid_cell(grid_x, grid_y):
            return False
        
        # Check cell and safety margin around it
        for dx in range(-safety_margin, safety_margin + 1):
            for dy in range(-safety_margin, safety_margin + 1):
                check_x, check_y = grid_x + dx, grid_y + dy
                if self.is_valid_cell(check_x, check_y):
                    if self.occupancy_grid[check_y, check_x] >= 50:
                        return False
                else:
                    return False  # Outside grid bounds
        return True

    def astar_search_robust(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Improved A* with better goal handling and fallback strategies"""
        
        # Validate start and goal with safety margin
        if not self.is_cell_free(*start, safety_margin=1) or not self.is_cell_free(*goal, safety_margin=1):
            self.get_logger().warn(f'Start {start} or goal {goal} not safe, trying to find nearest free cells')
            
            # Try to find nearest free cell to goal
            goal = self.find_nearest_free_cell(goal)
            if goal is None:
                self.get_logger().error('Cannot find free goal cell')
                return None
            
            # Check start again
            if not self.is_cell_free(*start, safety_margin=1):
                start = self.find_nearest_free_cell(start)
                if start is None:
                    self.get_logger().error('Cannot find free start cell')
                    return None
        
        # Rest of A* algorithm (same as before but with improved neighbor checking)
        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}
        closed_set = set()
        
        iterations = 0
        
        while open_set and iterations < self.max_planning_iterations:
            iterations += 1
            
            _, current_g, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Check if we're close enough to goal (within tolerance)
            if self.heuristic(current, goal) <= 2.0:  # Within 2 grid cells
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                
                self.get_logger().info(f'A* found path: {len(path)} points, {iterations} iterations')
                return path
            
            for neighbor, move_cost in self.get_neighbors_safe(current):
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
        
        self.get_logger().warn(f'A* failed after {iterations} iterations')
        return None

    def get_neighbors_safe(self, cell: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        """Get valid neighbors with safety checking"""
        x, y = cell
        neighbors = []
        
        # 8-connected grid
        directions = [
            (-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
            (0, -1, 1.0),                         (0, 1, 1.0),
            (1, -1, math.sqrt(2)),  (1, 0, 1.0),  (1, 1, math.sqrt(2))
        ]
        
        for dx, dy, cost in directions:
            new_x, new_y = x + dx, y + dy
            if self.is_cell_free(new_x, new_y, safety_margin=0):  # Use basic check for neighbors
                neighbors.append(((new_x, new_y), cost))
        
        return neighbors

    def find_nearest_free_cell(self, target: Tuple[int, int], max_radius: int = 10) -> Optional[Tuple[int, int]]:
        """Find nearest free cell to target location"""
        target_x, target_y = target
        
        for radius in range(1, max_radius + 1):
            # Check cells in expanding square around target
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    # Only check perimeter of current radius
                    if abs(dx) == radius or abs(dy) == radius:
                        check_x, check_y = target_x + dx, target_y + dy
                        if self.is_cell_free(check_x, check_y, safety_margin=1):
                            self.get_logger().info(f'Found free cell at ({check_x}, {check_y}), radius {radius}')
                            return (check_x, check_y)
        
        return None

    def emergency_stop_and_replan(self):
        """Emergency stop when planning fails repeatedly"""
        self.get_logger().warn('Multiple planning failures - emergency stop and replan')
        
        # Stop the robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Clear current path
        self.path_following = False
        self.current_path = []
        
        # Wait a bit for obstacles to potentially move
        time.sleep(2.0)
        
        # Try planning again
        self.create_timer(3.0, lambda: self.plan_path_to_goal())

    # Modify your plan_path_to_goal method to use the robust version:
    def plan_path_to_goal(self):
        """Plan path using robust A*"""
        if not self.current_pose or not self.has_goal or self.planning_active:
            return
        
        self.planning_active = True
        self.planning_failures = getattr(self, 'planning_failures', 0)
        
        try:
            start_gx, start_gy = self.world_to_grid(
                self.current_pose.position.x, self.current_pose.position.y)
            goal_gx, goal_gy = self.world_to_grid(self.target_x, self.target_y)
            
            self.get_logger().info(f'Planning: ({start_gx},{start_gy}) -> ({goal_gx},{goal_gy})')
            
            # Use robust A* search
            grid_path = self.astar_search_robust((start_gx, start_gy), (goal_gx, goal_gy))
            
            if grid_path and len(grid_path) > 1:
                # Reset failure counter on success
                self.planning_failures = 0
                
                # Smooth path
                smoothed_path = self.smooth_path(grid_path)
                
                # Convert to world coordinates
                self.current_path = []
                for gx, gy in smoothed_path[1:]:  # Skip first point (current position)
                    wx, wy = self.grid_to_world(gx, gy)
                    self.current_path.append((wx, wy))
                
                self.current_path_index = 0
                self.path_following = True
                
                self.get_logger().info(f'Path planned: {len(grid_path)} -> {len(smoothed_path)} -> {len(self.current_path)} points')
                self.publish_path()
            else:
                self.planning_failures += 1
                self.get_logger().error(f'A* planning failed! (Failure #{self.planning_failures})')
                
                # Emergency stop after multiple failures
                if self.planning_failures >= 5:
                    self.emergency_stop_and_replan()
                    self.planning_failures = 0
                    
        except Exception as e:
            self.get_logger().error(f'Planning error: {str(e)}')
            self.planning_failures += 1
        finally:
            self.planning_active = False
    
    def setup_boundary_obstacles(self):
        """Create boundary walls in the occupancy grid"""
        # Add boundary walls to occupancy grid
        boundary_thickness = 2  # grid cells
        
        # Convert boundary to grid coordinates
        min_gx, min_gy = self.world_to_grid(self.boundary_min_x, self.boundary_min_y)
        max_gx, max_gy = self.world_to_grid(self.boundary_max_x, self.boundary_max_y)
        
        # Create walls
        for gx in range(max(0, min_gx - boundary_thickness), min(self.map_width, max_gx + boundary_thickness + 1)):
            for gy in range(max(0, min_gy - boundary_thickness), min(self.map_height, max_gy + boundary_thickness + 1)):
                # Outside the boundary = obstacle
                if gx < min_gx or gx > max_gx or gy < min_gy or gy > max_gy:
                    self.occupancy_grid[gy, gx] = 100

    def is_within_bounds(self, x: float, y: float) -> bool:
        """Check if position is within the defined boundary"""
        return (self.boundary_min_x <= x <= self.boundary_max_x and 
                self.boundary_min_y <= y <= self.boundary_max_y)

    def get_next_waypoint(self):
        """Get the next waypoint in sequence"""
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        self.target_x, self.target_y = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index + 1}: ({self.target_x}, {self.target_y})')

    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int((world_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        world_x = grid_x * self.map_resolution + self.map_origin_x
        world_y = grid_y * self.map_resolution + self.map_origin_y
        return world_x, world_y

    def is_valid_cell(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid cell is within bounds"""
        return 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """A* heuristic function"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, cell: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        """Get valid neighbors with movement costs"""
        x, y = cell
        neighbors = []
        
        # 8-connected grid
        directions = [
            (-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
            (0, -1, 1.0),                         (0, 1, 1.0),
            (1, -1, math.sqrt(2)),  (1, 0, 1.0),  (1, 1, math.sqrt(2))
        ]
        
        for dx, dy, cost in directions:
            new_x, new_y = x + dx, y + dy
            if (self.is_valid_cell(new_x, new_y) and 
                self.occupancy_grid[new_y, new_x] < 50):  # Less strict threshold
                neighbors.append(((new_x, new_y), cost))
        
        return neighbors

    def astar_search(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* pathfinding algorithm"""
        if not self.is_valid_cell(*start) or not self.is_valid_cell(*goal):
            return None
        
        if (self.occupancy_grid[start[1], start[0]] >= 50 or 
            self.occupancy_grid[goal[1], goal[0]] >= 50):
            return None
        
        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}
        closed_set = set()
        
        iterations = 0
        
        while open_set and iterations < self.max_planning_iterations:
            iterations += 1
            
            _, current_g, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                
                self.get_logger().info(f'A* found path: {len(path)} points, {iterations} iterations')
                return path
            
            for neighbor, move_cost in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
        
        self.get_logger().warn(f'A* failed after {iterations} iterations')
        return None

    def smooth_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Simple path smoothing"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            j = min(len(path) - 1, i + 5)  # Look ahead maximum 5 points
            while j > i + 1:
                if self.line_of_sight(path[i], path[j]):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        
        return smoothed

    def line_of_sight(self, start: Tuple[int, int], end: Tuple[int, int]) -> bool:
        """Check line of sight using simple ray casting"""
        x0, y0 = start
        x1, y1 = end
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if not self.is_valid_cell(x, y) or self.occupancy_grid[y, x] >= 50:
                return False
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True

    def plan_path_to_goal(self):
        """Plan path using A*"""
        if not self.current_pose or not self.has_goal or self.planning_active:
            return
        
        self.planning_active = True
        
        try:
            start_gx, start_gy = self.world_to_grid(
                self.current_pose.position.x, self.current_pose.position.y)
            goal_gx, goal_gy = self.world_to_grid(self.target_x, self.target_y)
            
            self.get_logger().info(f'Planning: ({start_gx},{start_gy}) -> ({goal_gx},{goal_gy})')
            
            grid_path = self.astar_search((start_gx, start_gy), (goal_gx, goal_gy))
            
            if grid_path and len(grid_path) > 1:
                # Smooth path
                smoothed_path = self.smooth_path(grid_path)
                
                # Convert to world coordinates
                self.current_path = []
                for gx, gy in smoothed_path[1:]:  # Skip first point (current position)
                    wx, wy = self.grid_to_world(gx, gy)
                    self.current_path.append((wx, wy))
                
                self.current_path_index = 0
                self.path_following = True
                
                self.get_logger().info(f'Path planned: {len(grid_path)} -> {len(smoothed_path)} -> {len(self.current_path)} points')
                self.publish_path()
            else:
                self.get_logger().error('A* planning failed!')
                
        except Exception as e:
            self.get_logger().error(f'Planning error: {str(e)}')
        finally:
            self.planning_active = False

    def initial_planning(self):
        """Initial path planning after startup"""
        if self.current_pose:
            self.get_logger().info('Starting initial path planning...')
            self.plan_path_to_goal()
        else:
            # Retry if no pose yet
            self.create_timer(1.0, self.initial_planning)

    def follow_path(self) -> Twist:
        """Follow the planned path"""
        twist = Twist()
        
        if not self.current_path or self.current_path_index >= len(self.current_path):
            return twist
        
        # Get current target point
        target_wx, target_wy = self.current_path[self.current_path_index]
        
        dx = target_wx - self.current_pose.position.x
        dy = target_wy - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if waypoint reached
        if distance < self.path_point_tolerance:
            self.current_path_index += 1
            if self.current_path_index >= len(self.current_path):
                # Check if final goal reached
                final_dx = self.target_x - self.current_pose.position.x
                final_dy = self.target_y - self.current_pose.position.y
                final_distance = math.sqrt(final_dx*final_dx + final_dy*final_dy)
                
                if final_distance < self.waypoint_reached_tolerance:
                    self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached!')
                    self.path_following = False
                    
                    if self.autonomous_mode:
                        # Move to next waypoint
                        time.sleep(1.0)  # Brief pause
                        self.get_next_waypoint()
                        self.create_timer(1.0, lambda: self.plan_path_to_goal())
                    
                return twist
        
        # Calculate movement
        target_yaw = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        angle_diff = self.normalize_angle(target_yaw - current_yaw)
        
        if abs(angle_diff) > 0.3:
            # Turn more
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            twist.linear.x = 0.1
        else:
            # Move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = angle_diff * 2.0
        
        return twist

    def update_occupancy_grid(self):
        """Update occupancy grid from laser scan"""
        if not self.current_pose or not self.current_scan:
            return
        
        try:
            # Clear previous dynamic obstacles
            for gx, gy in self.dynamic_obstacles.copy():
                if self.is_valid_cell(gx, gy):
                    self.occupancy_grid[gy, gx] = 0
            self.dynamic_obstacles.clear()
            
            # Re-setup boundary (in case it was cleared)
            self.setup_boundary_obstacles()
            
            # Process laser scan
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            robot_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            
            angle = self.current_scan.angle_min
            
            for distance in self.current_scan.ranges:
                if (not (math.isinf(distance) or math.isnan(distance)) and 
                    0.1 < distance < self.obstacle_threshold):
                    
                    obs_x = robot_x + distance * math.cos(robot_yaw + angle)
                    obs_y = robot_y + distance * math.sin(robot_yaw + angle)
                    
                    # Only add obstacles within bounds
                    if self.is_within_bounds(obs_x, obs_y):
                        center_gx, center_gy = self.world_to_grid(obs_x, obs_y)
                        
                        # Inflate obstacle
                        for dx in range(-self.obstacle_inflation_radius, self.obstacle_inflation_radius + 1):
                            for dy in range(-self.obstacle_inflation_radius, self.obstacle_inflation_radius + 1):
                                if dx*dx + dy*dy <= self.obstacle_inflation_radius*self.obstacle_inflation_radius:
                                    gx, gy = center_gx + dx, center_gy + dy
                                    if self.is_valid_cell(gx, gy):
                                        self.occupancy_grid[gy, gx] = 75
                                        self.dynamic_obstacles.add((gx, gy))
                
                angle += self.current_scan.angle_increment
            
            self.publish_occupancy_grid()
            
        except Exception as e:
            self.get_logger().warn(f'Grid update error: {str(e)}')

    def check_replanning(self):
        """Check if replanning is needed"""
        if not self.path_following or not self.current_path or self.planning_active:
            return
        
        # Check if current path is blocked
        path_blocked = False
        for i in range(self.current_path_index, min(len(self.current_path), self.current_path_index + 3)):
            wx, wy = self.current_path[i]
            gx, gy = self.world_to_grid(wx, wy)
            if self.is_valid_cell(gx, gy) and self.occupancy_grid[gy, gx] >= 50:
                path_blocked = True
                break
        
        if path_blocked:
            self.get_logger().info('Path blocked - replanning!')
            self.plan_path_to_goal()

    def publish_path(self):
        """Publish path for visualization"""
        if not self.current_path:
            return
        
        try:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'odom'
            
            for wx, wy in self.current_path:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = wx
                pose_stamped.pose.position.y = wy
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)
            
            self.path_pub.publish(path_msg)
        except Exception as e:
            self.get_logger().warn(f'Path publish error: {str(e)}')

    def publish_occupancy_grid(self):
        """Publish occupancy grid"""
        try:
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = 'odom'
            
            grid_msg.info.resolution = self.map_resolution
            grid_msg.info.width = self.map_width
            grid_msg.info.height = self.map_height
            grid_msg.info.origin.position.x = self.map_origin_x
            grid_msg.info.origin.position.y = self.map_origin_y
            grid_msg.info.origin.orientation.w = 1.0
            
            grid_msg.data = self.occupancy_grid.flatten().tolist()
            self.map_pub.publish(grid_msg)
        except:
            pass

    def publish_markers(self):
        """Publish visualization markers for boundary and waypoints"""
        marker_array = MarkerArray()
        
        # Boundary marker
        boundary_marker = Marker()
        boundary_marker.header.frame_id = 'odom'
        boundary_marker.header.stamp = self.get_clock().now().to_msg()
        boundary_marker.ns = 'boundary'
        boundary_marker.id = 0
        boundary_marker.type = Marker.LINE_STRIP
        boundary_marker.action = Marker.ADD
        boundary_marker.scale.x = 0.05
        boundary_marker.color.r = 1.0
        boundary_marker.color.g = 0.0
        boundary_marker.color.b = 0.0
        boundary_marker.color.a = 1.0
        
        # Boundary points
        boundary_points = [
            (self.boundary_min_x, self.boundary_min_y),
            (self.boundary_max_x, self.boundary_min_y),
            (self.boundary_max_x, self.boundary_max_y),
            (self.boundary_min_x, self.boundary_max_y),
            (self.boundary_min_x, self.boundary_min_y)  # Close the loop
        ]
        
        for x, y in boundary_points:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.1
            boundary_marker.points.append(point)
        
        marker_array.markers.append(boundary_marker)
        
        # Waypoint markers
        for i, (wx, wy) in enumerate(self.waypoints):
            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = 'odom'
            waypoint_marker.header.stamp = self.get_clock().now().to_msg()
            waypoint_marker.ns = 'waypoints'
            waypoint_marker.id = i
            waypoint_marker.type = Marker.CYLINDER
            waypoint_marker.action = Marker.ADD
            waypoint_marker.pose.position.x = wx
            waypoint_marker.pose.position.y = wy
            waypoint_marker.pose.position.z = 0.1
            waypoint_marker.pose.orientation.w = 1.0
            waypoint_marker.scale.x = 0.4
            waypoint_marker.scale.y = 0.4
            waypoint_marker.scale.z = 0.2
            
            # Different colors for waypoints
            if i == self.current_waypoint_index:
                # Current target - bright green
                waypoint_marker.color.r = 0.0
                waypoint_marker.color.g = 1.0
                waypoint_marker.color.b = 0.0
            else:
                # Other waypoints - blue
                waypoint_marker.color.r = 0.0
                waypoint_marker.color.g = 0.0
                waypoint_marker.color.b = 1.0
            waypoint_marker.color.a = 0.8
            
            marker_array.markers.append(waypoint_marker)
        
        self.marker_pub.publish(marker_array)

    def control_loop(self):
        """Main control loop"""
        if not self.current_pose or not self.moving or not self.has_goal:
            return
        
        twist = Twist()
        
        if self.path_following:
            twist = self.follow_path()
        
        self.cmd_vel_pub.publish(twist)

    def print_status(self):
        """Print status information"""
        if self.current_pose:
            distance_to_target = math.sqrt(
                (self.target_x - self.current_pose.position.x)**2 + 
                (self.target_y - self.current_pose.position.y)**2
            )
            
            mode = "Path Following" if self.path_following else "Planning"
            self.get_logger().info(
                f'Status: Pos({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}) '
                f'-> Waypoint {self.current_waypoint_index + 1}({self.target_x:.1f}, {self.target_y:.1f}) '
                f'| Dist: {distance_to_target:.2f}m | Mode: {mode}'
            )

    def command_callback(self, msg):
        """Handle commands"""
        parts = msg.data.split()
        
        if parts[0] == 'stop':
            self.moving = False
            self.autonomous_mode = False
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Stopped')
        
        elif parts[0] == 'start':
            self.moving = True
            self.autonomous_mode = True
            self.get_logger().info('Started autonomous waypoint navigation')
            
        elif parts[0] == 'next':
            self.get_next_waypoint()
            self.plan_path_to_goal()
        
        elif parts[0] == 'replan':
            self.plan_path_to_goal()

    def odom_callback(self, msg):
        """Update current pose"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Update laser scan"""
        self.current_scan = msg

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw"""
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        )

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main():
    rclpy.init()
    controller = BoundedAStarNavigation()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
        twist = Twist()
        controller.cmd_vel_pub.publish(twist)
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()