Dynamic Obstacle Avoidance with A* Navigation
A comprehensive ROS2 implementation of autonomous robot navigation using A* pathfinding algorithm with real-time dynamic obstacle detection and avoidance.
🚀 Features

Bounded Area Navigation: Robot operates within a defined 6x6 meter boundary
A Pathfinding*: Efficient path planning with obstacle avoidance
Dynamic Obstacle Detection: Real-time laser scan processing for obstacle mapping
Waypoint Navigation: Autonomous navigation through predefined waypoints
Dynamic Obstacle Spawning: Simulated dynamic environment with moving obstacles
Path Replanning: Automatic replanning when obstacles block the current path
Visualization: RViz integration for path and map visualization

🏗️ Architecture
Core Components

A Navigation Controller* (astar_navigation_controller)

Main navigation logic using A* algorithm
Occupancy grid mapping from laser scans
Path following and replanning capabilities


Bounded Obstacle Spawner (obstacle_spawner)

Dynamic obstacle generation in Gazebo
Robot-aware spawning to prevent trapping
Automatic cleanup of old obstacles


Goal Setter (goal_setter)

Command interface for navigation control
Initial waypoint setup



📦 Dependencies

ROS2 Humble
TurtleBot3 packages
Gazebo
RViz2
Navigation2 (nav2)

bashsudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
🛠️ Installation

Clone the repository:

bashcd ~/ros2_ws/src
git clone https://github.com/yourusername/dynamic_obstacle_avoidance.git

Set TurtleBot3 model:

bashecho "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc

Build the workspace:

bashcd ~/ros2_ws
colcon build --packages-select dynamic_obstacle_avoidance
source install/setup.bash
🚀 Usage
Launch the Complete System
bashros2 launch dynamic_obstacle_avoidance complete_navigation.launch.py
This launches:

Gazebo simulation with TurtleBot3
A* navigation controller
Dynamic obstacle spawner
RViz visualization

Manual Control Commands
Control the robot using topic commands:
bash# Start autonomous navigation
ros2 topic pub /simple_nav_command std_msgs/String "data: start"

# Stop the robot
ros2 topic pub /simple_nav_command std_msgs/String "data: stop"

# Move to next waypoint
ros2 topic pub /simple_nav_command std_msgs/String "data: next"

# Force replanning
ros2 topic pub /simple_nav_command std_msgs/String "data: replan"
Individual Node Launch
Launch components separately for debugging:
bash# Launch only Gazebo with TurtleBot3
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Launch navigation controller
ros2 run dynamic_obstacle_avoidance astar_navigation_controller

# Launch obstacle spawner
ros2 run dynamic_obstacle_avoidance obstacle_spawner

# Launch goal setter
ros2 run dynamic_obstacle_avoidance goal_setter
📊 Configuration
Navigation Parameters
Edit parameters in the launch file or pass them at runtime:
pythonparameters=[{
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
Obstacle Spawning Parameters
pythonparameters=[{
    'spawn_interval': 4.0,      # seconds between spawns
    'delete_after': 12.0,       # obstacle lifetime
    'max_obstacles': 6,         # maximum concurrent obstacles
    'robot_clearance': 1.5      # minimum distance from robot
}]
🗺️ Navigation Behavior
Waypoint Sequence
The robot navigates through four waypoints in sequence:

Bottom-left: (-2.0, -2.0)
Bottom-right: (2.0, -2.0)
Top-right: (2.0, 2.0)
Top-left: (-2.0, 2.0)

Path Planning Strategy

Grid-based A*: 120x120 occupancy grid at 0.1m resolution
Obstacle Inflation: 3-cell radius inflation for safety
Path Smoothing: Line-of-sight optimization
Dynamic Replanning: 2-second interval checking for blocked paths

📈 Performance Characteristics

Planning Time: ~50-200ms for typical scenarios
Grid Resolution: 0.1m (configurable)
Maximum Planning Iterations: 8000
Replanning Frequency: Every 2 seconds
Robot Speed: 0.25 m/s linear, 0.6 rad/s angular

🔍 Visualization
RViz Topics

/planned_path: Current planned path
/dynamic_map: Occupancy grid with obstacles
/area_markers: Boundary and waypoint markers
/scan: Laser scan data

Debug Information
Monitor navigation status:
bashros2 topic echo /rosout | grep astar_navigation_controller
🐛 Troubleshooting
Common Issues

"A planning failed!" errors*

Check if robot is trapped by obstacles
Verify boundary parameters
Increase max_planning_iterations


Robot not moving

Ensure robot is within defined boundaries
Check if goal is reachable
Verify /cmd_vel topic is being published


Obstacles spawning on robot

Verify odometry topic (/odom) is publishing
Check robot_clearance parameter
Ensure obstacle spawner is receiving robot position



Debug Mode
Enable detailed logging:
bashros2 run dynamic_obstacle_avoidance astar_navigation_controller --ros-args --log-level debug
