#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import time
import random

class AutonomousMovementController(Node):
    def __init__(self):
        super().__init__('autonomous_movement_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.cmd_sub = self.create_subscription(
            String,
            '/simple_nav_command',
            self.command_callback,
            10
        )
        
        # State variables
        self.current_pose = None
        self.current_scan = None
        self.target_x = 0.0
        self.target_y = 0.0
        self.moving = True  # Start moving autonomously
        self.autonomous_mode = True  # Enable autonomous behavior
        
        # Obstacle avoidance state
        self.obstacle_detected = False
        self.avoidance_mode = False
        self.avoidance_direction = 1  # 1 for left, -1 for right
        self.avoidance_start_time = 0
        self.stuck_counter = 0
        self.last_position = None
        self.position_check_time = time.time()
        
        # Control parameters
        self.linear_speed = 0.25
        self.angular_speed = 0.6
        self.goal_tolerance = 0.5
        self.obstacle_threshold = 0.8  # Increased for better detection
        self.min_obstacle_distance = 0.5  # Minimum safe distance
        
        # Autonomous behavior parameters
        self.area_bounds = 4.0  # Stay within [-4, 4] area
        self.goal_timeout = 15.0  # Pick new goal if stuck for too long
        self.goal_start_time = time.time()
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Pick first random goal
        self.pick_new_goal()
        
        self.get_logger().info('Autonomous Movement Controller started')
        self.get_logger().info('Robot will move autonomously. Commands: "stop" or "start"')

    def pick_new_goal(self):
        """Pick a new random goal within the area bounds"""
        self.target_x = random.uniform(-self.area_bounds, self.area_bounds)
        self.target_y = random.uniform(-self.area_bounds, self.area_bounds)
        self.goal_start_time = time.time()
        self.stuck_counter = 0
        self.get_logger().info(f'New autonomous goal: ({self.target_x:.1f}, {self.target_y:.1f})')

    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose

    def pose_callback(self, msg):
        """Update current pose from AMCL (fallback)"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.current_scan = msg
        
        if len(msg.ranges) == 0:
            return
            
        # Analyze different sectors of the laser scan
        total_points = len(msg.ranges)
        
        # Front sector (±45 degrees)
        front_width = int(total_points * 0.25)  # 45 degrees on each side
        front_ranges = []
        
        # Left sector (45-135 degrees)
        left_start = int(total_points * 0.25)
        left_end = int(total_points * 0.75)
        left_ranges = []
        
        # Right sector (-45 to -135 degrees)
        right_ranges = []
        
        for i in range(total_points):
            distance = msg.ranges[i]
            if math.isinf(distance) or math.isnan(distance):
                continue
                
            # Front sector
            if i <= front_width or i >= total_points - front_width:
                front_ranges.append(distance)
            # Left sector
            elif left_start <= i <= left_end:
                left_ranges.append(distance)
            # Right sector
            else:
                right_ranges.append(distance)
        
        # Check for obstacles in each sector
        self.front_clear = not front_ranges or min(front_ranges) > self.obstacle_threshold
        self.left_clear = not left_ranges or min(left_ranges) > self.min_obstacle_distance
        self.right_clear = not right_ranges or min(right_ranges) > self.min_obstacle_distance
        
        # Overall obstacle detection
        if front_ranges:
            min_front_distance = min(front_ranges)
            self.obstacle_detected = min_front_distance < self.obstacle_threshold
        else:
            self.obstacle_detected = False

    def command_callback(self, msg):
        """Handle movement commands"""
        parts = msg.data.split()
        
        if len(parts) == 3 and parts[0] == 'go':
            try:
                self.target_x = float(parts[1])
                self.target_y = float(parts[2])
                self.moving = True
                self.autonomous_mode = False  # Manual goal
                self.get_logger().info(f'Manual goal: ({self.target_x}, {self.target_y})')
            except ValueError:
                self.get_logger().error('Invalid coordinates')
        
        elif parts[0] == 'stop':
            self.moving = False
            self.autonomous_mode = False
            self.stop_robot()
            self.get_logger().info('Stopping robot')
            
        elif parts[0] == 'start':
            self.moving = True
            self.autonomous_mode = True
            self.pick_new_goal()
            self.get_logger().info('Starting autonomous mode')

    def check_if_stuck(self):
        """Check if robot is stuck and hasn't moved much"""
        current_time = time.time()
        
        if self.current_pose and current_time - self.position_check_time > 2.0:
            if self.last_position:
                dx = self.current_pose.position.x - self.last_position.position.x
                dy = self.current_pose.position.y - self.last_position.position.y
                distance_moved = math.sqrt(dx*dx + dy*dy)
                
                if distance_moved < 0.1:  # Hasn't moved much in 2 seconds
                    self.stuck_counter += 1
                    if self.stuck_counter > 3:  # Stuck for 6+ seconds
                        self.get_logger().info('Robot seems stuck, picking new goal')
                        self.pick_new_goal()
                        return True
                else:
                    self.stuck_counter = 0
            
            self.last_position = self.current_pose
            self.position_check_time = current_time
        
        return False

    def smart_obstacle_avoidance(self):
        """Improved obstacle avoidance logic"""
        twist = Twist()
        
        # If just detected obstacle, choose avoidance direction
        if not self.avoidance_mode:
            self.avoidance_mode = True
            self.avoidance_start_time = time.time()
            # Choose direction based on which side is more clear
            if self.left_clear and not self.right_clear:
                self.avoidance_direction = 1  # Turn left
            elif self.right_clear and not self.left_clear:
                self.avoidance_direction = -1  # Turn right
            else:
                # Both sides similar, choose randomly but stick with it
                self.avoidance_direction = random.choice([1, -1])
            
            self.get_logger().info(f'Starting obstacle avoidance, turning {"left" if self.avoidance_direction > 0 else "right"}')
        
        # Continue avoidance maneuver
        avoidance_time = time.time() - self.avoidance_start_time
        
        if avoidance_time < 1.5:  # Turn for 1.5 seconds
            twist.angular.z = self.avoidance_direction * self.angular_speed
            twist.linear.x = 0.1  # Slow forward while turning
        elif avoidance_time < 3.0:  # Move forward for 1.5 seconds
            twist.linear.x = self.linear_speed * 0.7
            twist.angular.z = 0.0
        else:  # End avoidance maneuver
            self.avoidance_mode = False
            self.get_logger().info('Ending obstacle avoidance')
        
        return twist

    def control_loop(self):
        """Main control loop"""
        if not self.moving or not self.current_pose:
            return
        
        # Check if goal timed out (autonomous mode only)
        if self.autonomous_mode:
            goal_age = time.time() - self.goal_start_time
            if goal_age > self.goal_timeout:
                self.get_logger().info('Goal timeout, picking new goal')
                self.pick_new_goal()
                return
        
        # Check if stuck
        if self.check_if_stuck():
            return
        
        # Calculate distance and angle to target
        dx = self.target_x - self.current_pose.position.x
        dy = self.target_y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if goal reached
        if distance < self.goal_tolerance:
            if self.autonomous_mode:
                self.get_logger().info('Goal reached! Picking new goal...')
                self.pick_new_goal()
                return
            else:
                self.get_logger().info('Manual goal reached!')
                self.moving = False
                self.stop_robot()
                return
        
        # Create Twist message
        twist = Twist()
        
        # Obstacle avoidance takes priority
        if self.obstacle_detected or self.avoidance_mode:
            twist = self.smart_obstacle_avoidance()
        else:
            # Normal movement toward goal
            target_yaw = math.atan2(dy, dx)
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            angle_diff = self.normalize_angle(target_yaw - current_yaw)
            
            if abs(angle_diff) > 0.2:  # Need to turn
                twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                twist.linear.x = 0.1  # Slow forward while turning
            else:  # Move forward
                twist.linear.x = self.linear_speed
                twist.angular.z = angle_diff * 0.5  # Gentle steering
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
        
        # Periodic status update
        if int(time.time()) % 5 == 0:
            mode = "AUTO" if self.autonomous_mode else "MANUAL"
            self.get_logger().info(
                f'[{mode}] Pos: ({self.current_pose.position.x:.1f}, {self.current_pose.position.y:.1f}) '
                f'-> Goal: ({self.target_x:.1f}, {self.target_y:.1f}) '
                f'Dist: {distance:.1f}m',
                throttle_duration_sec=5.0
            )

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

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
    controller = AutonomousMovementController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()