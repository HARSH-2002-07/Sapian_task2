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
                # Launch bounded A* navigation controller
                Node(
                    package='dynamic_obstacle_avoidance',
                    executable='astar_navigation_controller',
                    name='astar_navigation_controller',
                    output='screen',
                    parameters=[{
                        'boundary_min_x': -3.0,
                        'boundary_max_x': 3.0,
                        'boundary_min_y': -3.0,
                        'boundary_max_y': 3.0,
                        'linear_speed': 0.25,
                        'angular_speed': 0.6,
                        'obstacle_threshold': 1.5,
                        'goal_tolerance': 0.3,
                        'waypoint_reached_tolerance': 0.4
                    }]
                ),
                
                # Launch bounded obstacle spawner
                Node(
                    package='dynamic_obstacle_avoidance',
                    executable='obstacle_spawner',
                    name='obstacle_spawner',
                    output='screen',
                    parameters=[{
                        'spawn_interval': 4.0,
                        'delete_after': 12.0,
                        'max_obstacles': 6
                    }]
                )
            ]
        ),
        
        # Launch RViz with navigation visualization
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