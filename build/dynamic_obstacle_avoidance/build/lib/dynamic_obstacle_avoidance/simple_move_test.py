#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleMoveTest(Node):
    def __init__(self):
        super().__init__('simple_move_test')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Move forward for 3 seconds, then turn for 2 seconds
        self.timer = self.create_timer(0.1, self.move_robot)
        self.start_time = time.time()
        self.phase = 'forward'  # 'forward' -> 'turn' -> 'stop'
        
        self.get_logger().info('Simple movement test started')

    def move_robot(self):
        twist = Twist()
        elapsed = time.time() - self.start_time
        
        if self.phase == 'forward' and elapsed < 3.0:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info(f'Moving forward... {elapsed:.1f}s')
            
        elif self.phase == 'forward':
            self.phase = 'turn'
            self.start_time = time.time()
            
        elif self.phase == 'turn' and elapsed < 2.0:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().info(f'Turning... {elapsed:.1f}s')
            
        elif self.phase == 'turn':
            self.phase = 'stop'
            
        else:  # stop phase
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Test complete - robot stopped')
        
        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = SimpleMoveTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        twist = Twist()
        node.publisher.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()