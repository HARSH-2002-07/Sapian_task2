#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    return LaunchDescription([
        # Launch Gazebo with empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(turtlebot3_gazebo_dir, 'launch'),
                '/empty_world.launch.py'
            ])
        ),
        
        # Wait 5 seconds for Gazebo to fully load
        TimerAction(
            period=5.0,
            actions=[
                # Launch A* navigation controller (replaces simple_controller)
                Node(
                    package='dynamic_obstacle_avoidance',  # Your package name
                    executable='astar_navigation_controller',  # New A* executable
                    name='astar_navigation_controller',
                    output='screen',
                    parameters=[{
                        'linear_speed': 0.25,
                        'angular_speed': 0.6,
                        'obstacle_threshold': 0.8,
                        'area_bounds': 4.0,
                        'replan_interval': 2.0,
                        'goal_tolerance': 0.3
                    }]
                ),
                
                # Launch obstacle spawner (unchanged)
                Node(
                    package='dynamic_obstacle_avoidance',
                    executable='obstacle_spawner',
                    name='dynamic_obstacle_spawner',
                    output='screen',
                    parameters=[{
                        'spawn_interval': 5.0,
                        'delete_after': 15.0
                    }]
                ),
                
                # Goal setter node for easy testing
                Node(
                    package='dynamic_obstacle_avoidance',
                    executable='goal_setter',
                    name='goal_setter',
                    output='screen'
                )
            ]
        ),
        
        # Launch RViz with custom config for A* visualization
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['rviz2', '-d', os.path.join(
                        get_package_share_directory('dynamic_obstacle_avoidance'),
                        'rviz', 'astar_navigation.rviz'
                    )],
                    output='screen'
                )
            ]
        )
    ])