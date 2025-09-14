#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import random
import time
import os
from collections import deque

class DynamicObstacleSpawner(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_spawner')
        
        # Service clients for Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Wait for services to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting...')
        
        self.get_logger().info('Dynamic Obstacle Spawner initialized')
        
        # Keep track of spawned obstacles with timestamps
        self.spawned_obstacles = deque()  # Will store tuples of (name, spawn_time)
        self.obstacle_counter = 0
        
        # Timing parameters
        self.spawn_interval = 5.0   # Spawn every 5 seconds
        self.delete_after = 15.0    # Delete after 15 seconds
        
        # Timer to spawn obstacles periodically
        self.spawn_timer = self.create_timer(self.spawn_interval, self.spawn_random_obstacle)
        
        # Timer to check for obstacles to delete (check every 2 seconds)
        self.delete_timer = self.create_timer(2.0, self.check_obstacles_for_deletion)
        
        # Subscriber to manually trigger obstacle spawning
        self.spawn_subscriber = self.create_subscription(
            String,
            '/spawn_obstacle_command',
            self.spawn_obstacle_callback,
            10
        )
        
        self.get_logger().info(f'Spawning obstacles every {self.spawn_interval} seconds')
        self.get_logger().info(f'Deleting obstacles after {self.delete_after} seconds')

    def get_box_sdf(self, name, size_x=0.5, size_y=0.5, size_z=0.5):
        """Generate SDF for a box obstacle"""
        sdf = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>1 0 0 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        return sdf

    def get_cylinder_sdf(self, name, radius=0.3, height=0.5):
        """Generate SDF for a cylinder obstacle"""
        sdf = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
          <specular>0 0 1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        return sdf

    def spawn_obstacle(self, x, y, z=0.25, obstacle_type='box', name=None):
        """Spawn an obstacle at specified coordinates"""
        if name is None:
            name = f"dynamic_obstacle_{self.obstacle_counter}"
            self.obstacle_counter += 1
        
        # Create spawn request
        request = SpawnEntity.Request()
        request.name = name
        
        if obstacle_type == 'box':
            request.xml = self.get_box_sdf(name)
        elif obstacle_type == 'cylinder':
            request.xml = self.get_cylinder_sdf(name)
        
        # Set pose
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        request.initial_pose = pose
        
        # Call spawn service
        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda fut, obs_name=name: self.spawn_callback(fut, obs_name)
        )

    def spawn_callback(self, future, obstacle_name):
        """Handle spawn service response"""
        try:
            response = future.result()
            if response.success:
                # Record spawn time
                spawn_time = time.time()
                self.spawned_obstacles.append((obstacle_name, spawn_time))
                self.get_logger().info(f'Successfully spawned obstacle: {obstacle_name} (Total: {len(self.spawned_obstacles)})')
            else:
                self.get_logger().error(f'Failed to spawn obstacle: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Exception in spawn callback: {str(e)}')

    def delete_obstacle(self, name):
        """Delete an obstacle by name"""
        request = DeleteEntity.Request()
        request.name = name
        
        future = self.delete_client.call_async(request)
        future.add_done_callback(
            lambda fut, obs_name=name: self.delete_callback(fut, obs_name)
        )

    def delete_callback(self, future, obstacle_name):
        """Handle delete service response"""
        try:
            response = future.result()
            if response.success:
                # Remove from tracking list
                self.spawned_obstacles = deque([
                    (name, spawn_time) for name, spawn_time in self.spawned_obstacles 
                    if name != obstacle_name
                ])
                self.get_logger().info(f'Successfully deleted obstacle: {obstacle_name} (Remaining: {len(self.spawned_obstacles)})')
            else:
                self.get_logger().error(f'Failed to delete obstacle: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Exception in delete callback: {str(e)}')

    def check_obstacles_for_deletion(self):
        """Check if any obstacles should be deleted based on age"""
        current_time = time.time()
        obstacles_to_delete = []
        
        for obstacle_name, spawn_time in self.spawned_obstacles:
            age = current_time - spawn_time
            if age >= self.delete_after:
                obstacles_to_delete.append(obstacle_name)
        
        # Delete old obstacles
        for obstacle_name in obstacles_to_delete:
            self.get_logger().info(f'Deleting obstacle {obstacle_name} after {self.delete_after} seconds')
            self.delete_obstacle(obstacle_name)

    def spawn_random_obstacle(self):
        """Spawn obstacle at random location"""
        # Define safe spawn area (avoid robot start position)
        x = random.uniform(-3.0, 3.0)
        y = random.uniform(-3.0, 3.0)
        
        # Avoid spawning too close to origin (robot start position)
        if abs(x) < 1.0 and abs(y) < 1.0:
            x = 1.5 if x >= 0 else -1.5
            y = 1.5 if y >= 0 else -1.5
        
        obstacle_type = random.choice(['box', 'cylinder'])
        self.get_logger().info(f'Spawning {obstacle_type} at ({x:.1f}, {y:.1f})')
        self.spawn_obstacle(x, y, obstacle_type=obstacle_type)

    def spawn_obstacle_callback(self, msg):
        """Handle manual obstacle spawn commands"""
        command = msg.data.split()
        
        if len(command) >= 3:
            try:
                x = float(command[1])
                y = float(command[2])
                obstacle_type = command[0] if command[0] in ['box', 'cylinder'] else 'box'
                
                self.spawn_obstacle(x, y, obstacle_type=obstacle_type)
                self.get_logger().info(f'Manual spawn: {obstacle_type} at ({x}, {y})')
            except ValueError:
                self.get_logger().error('Invalid command format. Use: "box/cylinder x y"')

    def clear_all_obstacles(self):
        """Remove all spawned obstacles"""
        obstacles_copy = list(self.spawned_obstacles)  # Create copy to avoid modification during iteration
        for obstacle_name, spawn_time in obstacles_copy:
            self.delete_obstacle(obstacle_name)

def main(args=None):
    rclpy.init(args=args)
    spawner = DynamicObstacleSpawner()
    
    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        spawner.get_logger().info('Shutting down obstacle spawner...')
        spawner.clear_all_obstacles()
    finally:
        spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()