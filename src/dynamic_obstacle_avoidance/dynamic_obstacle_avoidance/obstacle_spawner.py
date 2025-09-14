#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import random
import time
import math

class BoundedObstacleSpawner(Node):
    def __init__(self):
        super().__init__('bounded_obstacle_spawner')
        
        # Bounded area parameters (matching navigation controller)
        self.boundary_min_x = -3.0
        self.boundary_max_x = 3.0
        self.boundary_min_y = -3.0
        self.boundary_max_y = 3.0
        
        # Keep away from waypoints
        self.waypoints = [
            (-2.0, -2.0),  # Bottom-left
            (2.0, -2.0),   # Bottom-right  
            (2.0, 2.0),    # Top-right
            (-2.0, 2.0),   # Top-left
        ]
        self.waypoint_clearance = 0.8  # meters
        
        # Robot avoidance parameters
        self.robot_clearance = 1.5  # meters - minimum distance from robot
        self.robot_position = None
        
        # Spawning parameters
        self.spawn_interval = 4.0  # seconds
        self.delete_after = 12.0   # seconds
        self.max_obstacles = 6
        
        # Gazebo service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Subscribe to robot odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Track spawned obstacles
        self.spawned_obstacles = {}  # {name: (spawn_time, x, y)}
        self.obstacle_counter = 0
        self.obstacle_size = 0.4  # Obstacle clearance radius
        
        # Box model (SDF format)
        self.box_sdf = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="dynamic_box">
    <pose>0 0 0.1 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.083</iyy>
          <iyz>0.0</iyz>
          <izz>0.083</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>'''
        
        # Wait for services
        self.get_logger().info('Waiting for Gazebo services...')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting...')
        
        # Wait for robot position before starting
        self.get_logger().info('Waiting for robot position...')
        while self.robot_position is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Timers
        self.spawn_timer = self.create_timer(self.spawn_interval, self.spawn_obstacle)
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_obstacles)
        
        self.get_logger().info('Bounded Obstacle Spawner initialized')
        self.get_logger().info(f'Spawn area: ({self.boundary_min_x},{self.boundary_min_y}) to ({self.boundary_max_x},{self.boundary_max_y})')
        self.get_logger().info(f'Robot clearance: {self.robot_clearance}m, Waypoint clearance: {self.waypoint_clearance}m')
        self.get_logger().info(f'Spawning every {self.spawn_interval}s, deleting after {self.delete_after}s')

    def odom_callback(self, msg):
        """Update robot position"""
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def is_valid_spawn_location(self, x: float, y: float) -> bool:
        """Check if location is valid for spawning (within bounds, away from waypoints and robot)"""
        # Check if within boundary with margin
        margin = 0.3
        if not (self.boundary_min_x + margin <= x <= self.boundary_max_x - margin and 
                self.boundary_min_y + margin <= y <= self.boundary_max_y - margin):
            return False
        
        # Check distance from robot (most important!)
        if self.robot_position:
            robot_x, robot_y = self.robot_position
            robot_distance = math.sqrt((x - robot_x)**2 + (y - robot_y)**2)
            if robot_distance < self.robot_clearance:
                return False
        
        # Check distance from waypoints
        for wx, wy in self.waypoints:
            waypoint_distance = math.sqrt((x - wx)**2 + (y - wy)**2)
            if waypoint_distance < self.waypoint_clearance:
                return False
        
        # Check distance from existing obstacles
        for obstacle_name, (spawn_time, obs_x, obs_y) in self.spawned_obstacles.items():
            obstacle_distance = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if obstacle_distance < self.obstacle_size * 2:  # Don't spawn too close to existing obstacles
                return False
        
        return True

    def generate_spawn_position(self) -> tuple:
        """Generate a valid spawn position with multiple attempts"""
        max_attempts = 100  # Increased attempts
        
        for attempt in range(max_attempts):
            # Generate random position within bounds
            x = random.uniform(self.boundary_min_x + 0.5, self.boundary_max_x - 0.5)
            y = random.uniform(self.boundary_min_y + 0.5, self.boundary_max_y - 0.5)
            
            if self.is_valid_spawn_location(x, y):
                self.get_logger().debug(f'Found valid position after {attempt + 1} attempts: ({x:.2f}, {y:.2f})')
                return x, y
        
        # If still no valid position, try safer zones away from current robot position
        if self.robot_position:
            robot_x, robot_y = self.robot_position
            safe_zones = [
                (self.boundary_min_x + 0.5, self.boundary_min_y + 0.5),  # Bottom-left corner
                (self.boundary_max_x - 0.5, self.boundary_min_y + 0.5),  # Bottom-right corner
                (self.boundary_max_x - 0.5, self.boundary_max_y - 0.5),  # Top-right corner
                (self.boundary_min_x + 0.5, self.boundary_max_y - 0.5),  # Top-left corner
            ]
            
            # Sort safe zones by distance from robot (furthest first)
            safe_zones.sort(key=lambda pos: math.sqrt((pos[0] - robot_x)**2 + (pos[1] - robot_y)**2), reverse=True)
            
            for base_x, base_y in safe_zones:
                for _ in range(20):
                    # Add small random offset around safe zone
                    x = base_x + random.uniform(-0.3, 0.3)
                    y = base_y + random.uniform(-0.3, 0.3)
                    
                    if self.is_valid_spawn_location(x, y):
                        self.get_logger().info(f'Using safe zone position: ({x:.2f}, {y:.2f})')
                        return x, y
        
        # Last resort - skip spawning this time
        self.get_logger().warn('Could not find valid spawn location, skipping this spawn cycle')
        return None, None

    def spawn_obstacle(self):
        """Spawn a new obstacle"""
        if len(self.spawned_obstacles) >= self.max_obstacles:
            self.get_logger().debug('Maximum obstacles reached, skipping spawn')
            return
        
        if not self.robot_position:
            self.get_logger().warn('No robot position available, skipping spawn')
            return
        
        try:
            # Generate position
            x, y = self.generate_spawn_position()
            
            # Skip if no valid position found
            if x is None or y is None:
                return
            
            # Create unique name
            self.obstacle_counter += 1
            obstacle_name = f'bounded_obstacle_{self.obstacle_counter}'
            
            # Create spawn request
            spawn_request = SpawnEntity.Request()
            spawn_request.name = obstacle_name
            spawn_request.xml = self.box_sdf
            spawn_request.robot_namespace = ''
            spawn_request.initial_pose = Pose()
            spawn_request.initial_pose.position.x = x
            spawn_request.initial_pose.position.y = y
            spawn_request.initial_pose.position.z = 0.1
            spawn_request.initial_pose.orientation.w = 1.0
            spawn_request.reference_frame = 'world'
            
            # Send request
            future = self.spawn_client.call_async(spawn_request)
            future.add_done_callback(lambda f, name=obstacle_name, pos=(x, y): self.spawn_callback(f, name, pos))
            
        except Exception as e:
            self.get_logger().error(f'Spawn error: {str(e)}')

    def spawn_callback(self, future, obstacle_name: str, position: tuple):
        """Handle spawn response"""
        try:
            response = future.result()
            if response.success:
                x, y = position
                self.spawned_obstacles[obstacle_name] = (time.time(), x, y)
                robot_dist = "Unknown"
                if self.robot_position:
                    robot_x, robot_y = self.robot_position
                    robot_dist = f"{math.sqrt((x - robot_x)**2 + (y - robot_y)**2):.2f}m"
                
                self.get_logger().info(f'Spawned: {obstacle_name} at ({x:.2f}, {y:.2f}) - Robot dist: {robot_dist} (Total: {len(self.spawned_obstacles)})')
            else:
                self.get_logger().error(f'Failed to spawn {obstacle_name}: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Spawn callback error: {str(e)}')

    def cleanup_obstacles(self):
        """Delete old obstacles"""
        current_time = time.time()
        obstacles_to_delete = []
        
        for obstacle_name, (spawn_time, x, y) in self.spawned_obstacles.items():
            if current_time - spawn_time > self.delete_after:
                obstacles_to_delete.append(obstacle_name)
        
        for obstacle_name in obstacles_to_delete:
            self.delete_obstacle(obstacle_name)

    def delete_obstacle(self, obstacle_name: str):
        """Delete a specific obstacle"""
        try:
            delete_request = DeleteEntity.Request()
            delete_request.name = obstacle_name
            
            future = self.delete_client.call_async(delete_request)
            future.add_done_callback(lambda f, name=obstacle_name: self.delete_callback(f, name))
            
        except Exception as e:
            self.get_logger().error(f'Delete error for {obstacle_name}: {str(e)}')

    def delete_callback(self, future, obstacle_name: str):
        """Handle delete response"""
        try:
            response = future.result()
            if response.success:
                if obstacle_name in self.spawned_obstacles:
                    del self.spawned_obstacles[obstacle_name]
                self.get_logger().info(f'Deleted obstacle: {obstacle_name} (Remaining: {len(self.spawned_obstacles)})')
            else:
                self.get_logger().warn(f'Failed to delete {obstacle_name}: {response.status_message}')
                # Remove from tracking anyway
                if obstacle_name in self.spawned_obstacles:
                    del self.spawned_obstacles[obstacle_name]
        except Exception as e:
            self.get_logger().error(f'Delete callback error: {str(e)}')

def main():
    rclpy.init()
    spawner = BoundedObstacleSpawner()
    
    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        spawner.get_logger().info('Shutting down obstacle spawner...')
    finally:
        # Clean up all obstacles
        spawner.get_logger().info('Cleaning up obstacles...')
        for obstacle_name in list(spawner.spawned_obstacles.keys()):
            spawner.delete_obstacle(obstacle_name)
        time.sleep(1.0)  # Wait for deletions
        spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()