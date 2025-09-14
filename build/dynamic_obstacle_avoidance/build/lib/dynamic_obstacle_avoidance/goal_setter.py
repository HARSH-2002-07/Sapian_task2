#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys

class GoalSetter(Node):
    def __init__(self):
        super().__init__('goal_setter')
        
        # Publisher for navigation commands
        self.nav_cmd_pub = self.create_publisher(String, '/simple_nav_command', 10)
        
        # Publisher for obstacle commands  
        self.obs_cmd_pub = self.create_publisher(String, '/spawn_obstacle_command', 10)
        
        self.get_logger().info('Goal Setter ready!')
        self.print_help()
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def print_help(self):
        """Print available commands"""
        print("\n=== A* Navigation Commands ===")
        print("Navigation:")
        print("  go <x> <y>     - Navigate to position (x, y)")  
        print("  stop           - Stop navigation")
        print("  start          - Start autonomous mode")
        print("\nObstacle Control:")
        print("  box <x> <y>    - Spawn box obstacle at (x, y)")
        print("  cylinder <x> <y> - Spawn cylinder obstacle at (x, y)")
        print("\nOther:")
        print("  help           - Show this help")
        print("  quit           - Exit program")
        print("\nExamples:")
        print("  go 2 3         - Navigate to (2, 3)")
        print("  box 1 1        - Spawn box obstacle at (1, 1)")
        print("  cylinder -2 2  - Spawn cylinder at (-2, 2)")
        print("=====================================\n")

    def input_loop(self):
        """Handle user input in separate thread"""
        while rclpy.ok():
            try:
                user_input = input("Enter command: ").strip().lower()
                
                if not user_input:
                    continue
                    
                if user_input == 'quit' or user_input == 'exit':
                    self.get_logger().info('Shutting down goal setter...')
                    break
                
                if user_input == 'help':
                    self.print_help()
                    continue
                
                parts = user_input.split()
                command = parts[0]
                
                # Navigation commands
                if command in ['go', 'stop', 'start']:
                    msg = String()
                    msg.data = user_input
                    self.nav_cmd_pub.publish(msg)
                    self.get_logger().info(f'Sent navigation command: {user_input}')
                
                # Obstacle spawn commands
                elif command in ['box', 'cylinder']:
                    if len(parts) == 3:
                        try:
                            x = float(parts[1])
                            y = float(parts[2])
                            msg = String()
                            msg.data = user_input
                            self.obs_cmd_pub.publish(msg)
                            self.get_logger().info(f'Spawning {command} at ({x}, {y})')
                        except ValueError:
                            print("Error: Invalid coordinates")
                    else:
                        print(f"Error: {command} command needs x and y coordinates")
                        print(f"Usage: {command} <x> <y>")
                
                else:
                    print(f"Unknown command: {command}")
                    print("Type 'help' for available commands")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error processing input: {e}")

def main():
    rclpy.init()
    
    goal_setter = GoalSetter()
    
    try:
        # Use MultiThreadedExecutor to handle callbacks while input thread runs
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(goal_setter)
        executor.spin()
    except KeyboardInterrupt:
        goal_setter.get_logger().info('Goal setter interrupted')
    finally:
        goal_setter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()