#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time

class TopicDebugger(Node):
    def __init__(self):
        super().__init__('topic_debugger')
        
        # Track what we've received
        self.odom_received = False
        self.scan_received = False
        self.cmd_vel_published = False
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Publisher to test cmd_vel
        self.test_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for status and test
        self.status_timer = self.create_timer(2.0, self.print_status)
        self.test_timer = self.create_timer(5.0, self.test_movement)
        
        self.get_logger().info('Topic Debugger started - monitoring robot topics...')

    def odom_callback(self, msg):
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f'✓ ODOM received! Position: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})')

    def scan_callback(self, msg):
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info(f'✓ LASER SCAN received! {len(msg.ranges)} points')

    def cmd_callback(self, msg):
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(f'📡 CMD_VEL: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

    def print_status(self):
        """Print current status"""
        status = "=== TOPIC STATUS ==="
        status += f"\n📍 Odometry: {'✓ OK' if self.odom_received else '❌ NO DATA'}"
        status += f"\n🔍 Laser Scan: {'✓ OK' if self.scan_received else '❌ NO DATA'}"
        
        self.get_logger().info(status)

    def test_movement(self):
        """Send test movement command"""
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.1
        
        self.test_pub.publish(twist)
        self.get_logger().info('🚀 Sending test movement command...')
        
        # Stop after 1 second
        self.create_timer(1.0, lambda: self.test_pub.publish(Twist()))

def main():
    rclpy.init()
    debugger = TopicDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        debugger.get_logger().info('Debugger stopped')
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()