#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class GoalSetter(Node):
    def __init__(self):
        super().__init__('goal_setter')
        
        # Publisher for navigation commands
        self.cmd_pub = self.create_publisher(String, '/simple_nav_command', 10)
        
        # Timer for sending commands
        self.create_timer(2.0, self.send_initial_command)
        
        self.get_logger().info('Goal Setter Node started')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  ros2 topic pub /simple_nav_command std_msgs/String "data: start"')
        self.get_logger().info('  ros2 topic pub /simple_nav_command std_msgs/String "data: stop"')
        self.get_logger().info('  ros2 topic pub /simple_nav_command std_msgs/String "data: next"')
        self.get_logger().info('  ros2 topic pub /simple_nav_command std_msgs/String "data: replan"')

    def send_initial_command(self):
        """Send start command after initialization"""
        msg = String()
        msg.data = 'start'
        self.cmd_pub.publish(msg)
        self.get_logger().info('Sent initial start command')
        
        # Only send once
        self.destroy_timer(self.create_timer(2.0, self.send_initial_command))

def main():
    rclpy.init()
    goal_setter = GoalSetter()
    
    try:
        rclpy.spin(goal_setter)
    except KeyboardInterrupt:
        goal_setter.get_logger().info('Goal setter shutting down...')
    finally:
        goal_setter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()