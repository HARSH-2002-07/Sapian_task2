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
                # Launch autonomous movement controller
                Node(
                    package='dynamic_obstacle_avoidance',
                    executable='simple_controller',
                    name='autonomous_movement_controller',
                    output='screen',
                    parameters=[{
                        'linear_speed': 0.25,
                        'angular_speed': 0.6,
                        'obstacle_threshold': 0.8,
                        'area_bounds': 4.0
                    }]
                ),
                
                # Launch obstacle spawner
                Node(
                    package='dynamic_obstacle_avoidance',
                    executable='obstacle_spawner',
                    name='dynamic_obstacle_spawner',
                    output='screen',
                    parameters=[{
                        'spawn_interval': 5.0,
                        'delete_after': 15.0
                    }]
                )
            ]
        ),
        
        # Optional: Launch RViz for visualization
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['rviz2', '-d', os.path.join(
                        get_package_share_directory('turtlebot3_gazebo'),
                        'rviz', 'turtlebot3_gazebo.rviz'
                    )],
                    output='screen'
                )
            ]
        )
    ])