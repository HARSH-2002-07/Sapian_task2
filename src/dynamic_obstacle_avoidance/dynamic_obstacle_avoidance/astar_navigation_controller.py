#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped
import math
import time
import heapq
import numpy as np
from typing import List, Tuple, Optional

class AStarNavigationController(Node):
    def __init__(self):
        super().__init__('astar_navigation_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/dynamic_map', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(
            String, '/simple_nav_command', self.command_callback, 10)
        
        # Grid map parameters
        self.map_width = 100   # 10m at 0.1m resolution  
        self.map_height = 100
        self.map_resolution = 0.1  # meters per cell
        self.map_origin_x = -5.0   # meters
        self.map_origin_y = -5.0
        
        # Initialize occupancy grid
        self.occupancy_grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.static_obstacles = set()
        self.dynamic_obstacles = set()
        
        # Robot state
        self.current_pose = None
        self.current_scan = None
        self.target_x = 2.0  # Start with simple goal like debug version
        self.target_y = 0.0
        self.has_goal = True
        
        # Path planning and following
        self.current_path = []
        self.current_path_index = 0
        self.path_following = False
        self.last_replan_time = 0
        self.planning_active = False
        
        # Control parameters (matching debug version)
        self.linear_speed = 0.2  # Match debug version
        self.angular_speed = 0.5  # Match debug version
        self.goal_tolerance = 0.3
        self.path_point_tolerance = 0.2
        self.obstacle_inflation_radius = 2  # grid cells
        self.obstacle_threshold = 0.8
        
        # A* parameters
        self.replan_interval = 2.0  # seconds
        self.max_planning_iterations = 5000
        
        # Simple mode flags
        self.moving = True
        self.simple_navigation = True  # Start with simple navigation
        
        # Timers - match debug version frequency
        self.control_timer = self.create_timer(0.2, self.control_loop)  # Match debug frequency
        self.map_update_timer = self.create_timer(0.5, self.update_occupancy_grid)  # Less frequent
        self.status_timer = self.create_timer(2.0, self.print_status)  # Add status like debug
        
        self.get_logger().info('🚀 Simple A* Debug Controller started')
        self.get_logger().info(f'Target: ({self.target_x}, {self.target_y})')

    def pick_new_goal(self):
        """Pick a new random goal for autonomous navigation"""
        import random
        self.target_x = random.uniform(-2.0, 2.0)  # Smaller area initially
        self.target_y = random.uniform(-2.0, 2.0)
        self.has_goal = True
        self.simple_navigation = True  # Reset to simple navigation
        self.path_following = False
        self.get_logger().info(f'New goal: ({self.target_x:.1f}, {self.target_y:.1f})')

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
        """A* heuristic function (Euclidean distance)"""
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
                self.occupancy_grid[new_y, new_x] == 0):
                neighbors.append(((new_x, new_y), cost))
        
        return neighbors

    def astar_search(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* pathfinding algorithm"""
        if not self.is_valid_cell(*start) or not self.is_valid_cell(*goal):
            self.get_logger().warn(f'Invalid grid cells: start={start}, goal={goal}')
            return None
        
        if (self.occupancy_grid[start[1], start[0]] != 0 or 
            self.occupancy_grid[goal[1], goal[0]] != 0):
            self.get_logger().warn('Start or goal is occupied')
            return None
        
        # Priority queue: (f_score, g_score, cell)
        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        closed_set = set()
        
        iterations = 0
        
        while open_set and iterations < self.max_planning_iterations:
            iterations += 1
            
            # Get cell with lowest f_score
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
            
            # Explore neighbors
            for neighbor, move_cost in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], tentative_g, neighbor))
        
        self.get_logger().warn(f'A* failed to find path after {iterations} iterations')
        return None

    def plan_path_to_goal(self):
        """Plan path using A* algorithm"""
        if not self.current_pose or not self.has_goal or self.planning_active:
            return
        
        self.planning_active = True
        
        try:
            # Convert to grid coordinates
            start_gx, start_gy = self.world_to_grid(
                self.current_pose.position.x, self.current_pose.position.y)
            goal_gx, goal_gy = self.world_to_grid(self.target_x, self.target_y)
            
            self.get_logger().info(f'Planning: Start({start_gx},{start_gy}) -> Goal({goal_gx},{goal_gy})')
            
            start_time = time.time()
            grid_path = self.astar_search((start_gx, start_gy), (goal_gx, goal_gy))
            planning_time = time.time() - start_time
            
            if grid_path and len(grid_path) > 1:
                # Convert to world coordinates
                self.current_path = []
                for gx, gy in grid_path:
                    wx, wy = self.grid_to_world(gx, gy)
                    self.current_path.append((wx, wy))
                
                self.current_path_index = 0
                self.path_following = True
                self.simple_navigation = False
                self.last_replan_time = time.time()
                
                self.get_logger().info(f'Path planned in {planning_time:.3f}s: {len(grid_path)} points')
                self.publish_path()
            else:
                self.get_logger().error('A* planning failed - using simple navigation')
                self.path_following = False
                self.simple_navigation = True
                
        except Exception as e:
            self.get_logger().error(f'Planning error: {str(e)}')
            self.simple_navigation = True
            self.path_following = False
        finally:
            self.planning_active = False

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
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)
            
            self.path_pub.publish(path_msg)
        except Exception as e:
            self.get_logger().warn(f'Path publish error: {str(e)}')

    def update_occupancy_grid(self):
        """Update occupancy grid from laser scan"""
        if not self.current_pose or not self.current_scan:
            return
        
        try:
            # Clear previous dynamic obstacles
            for gx, gy in self.dynamic_obstacles:
                if self.is_valid_cell(gx, gy):
                    self.occupancy_grid[gy, gx] = 0
            self.dynamic_obstacles.clear()
            
            # Process laser scan
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y  
            robot_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            
            angle = self.current_scan.angle_min
            obstacle_detected = False
            
            for distance in self.current_scan.ranges:
                if not (math.isinf(distance) or math.isnan(distance) or distance > self.obstacle_threshold):
                    obstacle_detected = True
                    # Calculate obstacle position
                    obs_x = robot_x + distance * math.cos(robot_yaw + angle)
                    obs_y = robot_y + distance * math.sin(robot_yaw + angle)
                    
                    # Convert to grid and inflate
                    center_gx, center_gy = self.world_to_grid(obs_x, obs_y)
                    
                    for dx in range(-self.obstacle_inflation_radius, self.obstacle_inflation_radius + 1):
                        for dy in range(-self.obstacle_inflation_radius, self.obstacle_inflation_radius + 1):
                            if dx*dx + dy*dy <= self.obstacle_inflation_radius*self.obstacle_inflation_radius:
                                gx, gy = center_gx + dx, center_gy + dy
                                if self.is_valid_cell(gx, gy):
                                    self.occupancy_grid[gy, gx] = 100
                                    self.dynamic_obstacles.add((gx, gy))
                
                angle += self.current_scan.angle_increment
            
            # If obstacles detected and using simple navigation, try A* planning
            if obstacle_detected and self.simple_navigation and not self.planning_active:
                self.get_logger().info('Obstacles detected - attempting A* planning')
                self.plan_path_to_goal()
            
            # Publish occupancy grid
            self.publish_occupancy_grid()
            
        except Exception as e:
            self.get_logger().warn(f'Occupancy grid update error: {str(e)}')

    def publish_occupancy_grid(self):
        """Publish occupancy grid for visualization"""
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
        except Exception as e:
            self.get_logger().warn(f'Grid publish error: {str(e)}')

    def simple_navigation_control(self) -> Twist:
        """Simple navigation like debug version"""
        twist = Twist()
        
        if not self.current_pose or not self.has_goal:
            return twist
        
        # Calculate distance to goal
        dx = self.target_x - self.current_pose.position.x
        dy = self.target_y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if goal reached
        if distance < self.goal_tolerance:
            self.get_logger().info('🎯 GOAL REACHED!')
            self.pick_new_goal()  # Pick new goal like autonomous mode
            return twist
        
        # Simple navigation: turn toward goal, then move
        target_yaw = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        angle_diff = self.normalize_angle(target_yaw - current_yaw)
        
        if abs(angle_diff) > 0.2:  # Need to turn
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            twist.linear.x = 0.05  # Slow forward while turning
            self.get_logger().info(f'🔄 TURNING: angle_diff={angle_diff:.2f}')
        else:  # Move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = angle_diff * 1.0  # Small correction
            self.get_logger().info(f'➡️ MOVING FORWARD: distance={distance:.2f}')
        
        self.get_logger().info(f'📡 Published: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}')
        return twist

    def follow_path(self) -> Twist:
        """Generate commands to follow planned path"""
        twist = Twist()
        
        if not self.current_path or self.current_path_index >= len(self.current_path):
            self.get_logger().info('Path following complete - switching to simple navigation')
            self.path_following = False
            self.simple_navigation = True
            return twist
        
        # Get current target waypoint
        target_wx, target_wy = self.current_path[self.current_path_index]
        
        dx = target_wx - self.current_pose.position.x
        dy = target_wy - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if waypoint reached
        if distance < self.path_point_tolerance:
            self.current_path_index += 1
            if self.current_path_index >= len(self.current_path):
                # Goal reached
                self.get_logger().info('A* Goal reached!')
                self.path_following = False
                self.pick_new_goal()
                return twist
        
        # Calculate steering
        target_yaw = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        angle_diff = self.normalize_angle(target_yaw - current_yaw)
        
        if abs(angle_diff) > 0.3:
            # Turn towards target
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            twist.linear.x = 0.1
        else:
            # Move forward with steering
            twist.linear.x = self.linear_speed
            twist.angular.z = angle_diff * 2.0
        
        return twist

    def control_loop(self):
        """Main control loop - matches debug version structure"""
        if not self.current_pose:
            return
        
        if not self.moving or not self.has_goal:
            return
        
        twist = Twist()
        
        if self.path_following:
            twist = self.follow_path()
        elif self.simple_navigation:
            twist = self.simple_navigation_control()
        
        self.cmd_vel_pub.publish(twist)

    def print_status(self):
        """Print status every 2 seconds like debug version"""
        if self.current_pose:
            dx = self.target_x - self.current_pose.position.x
            dy = self.target_y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            mode = "A*" if self.path_following else "Simple"
            self.get_logger().info(f'📍 Status: Pos({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}) -> Goal({self.target_x}, {self.target_y}) | Dist: {distance:.2f}m | Mode: {mode} | Moving: {self.moving}')
        else:
            self.get_logger().info('📍 Status: NO ODOMETRY DATA')

    def command_callback(self, msg):
        """Handle navigation commands"""
        parts = msg.data.split()
        
        if len(parts) == 3 and parts[0] == 'go':
            try:
                self.target_x = float(parts[1])
                self.target_y = float(parts[2])
                self.has_goal = True
                self.moving = True
                self.simple_navigation = True
                self.path_following = False
                self.get_logger().info(f'Manual goal: ({self.target_x:.1f}, {self.target_y:.1f})')
            except ValueError:
                self.get_logger().error('Invalid coordinates')
        
        elif parts[0] == 'stop':
            self.moving = False
            self.path_following = False
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Stopped')
        
        elif parts[0] == 'start':
            self.moving = True
            self.has_goal = True
            self.simple_navigation = True
            self.get_logger().info('Started navigation')
        
        elif parts[0] == 'astar':
            if self.current_pose:
                self.get_logger().info('Manual A* planning requested')
                self.plan_path_to_goal()

    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose
        if not hasattr(self, '_first_odom'):
            self._first_odom = True
            self.get_logger().info(f'✅ ODOMETRY RECEIVED! Position: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})')

    def scan_callback(self, msg):
        """Update laser scan data"""
        self.current_scan = msg

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
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
    controller = AStarNavigationController()
    
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