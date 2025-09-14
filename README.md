# Dynamic Obstacle Avoidance Robot

A robot that moves around a room and avoids obstacles using A* pathfinding.

## What it does

- Robot moves between 4 waypoints in a square pattern
- Uses laser sensor to detect obstacles
- Plans new paths when obstacles block the way
- Spawns random obstacles to test avoidance

## Requirements

- ROS2 Humble
- TurtleBot3 packages
- Gazebo simulator

```bash
sudo apt install ros-humble-turtlebot3*
```

## How to run

1. Set up TurtleBot3:
```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

2. Build the project:
```bash
cd ~/ros2_ws
colcon build --packages-select dynamic_obstacle_avoidance
source install/setup.bash
```

3. Run everything:
```bash
ros2 launch dynamic_obstacle_avoidance complete_navigation.launch.py
```

## Control commands

```bash
# Start the robot
ros2 topic pub /simple_nav_command std_msgs/String "data: start"

# Stop the robot
ros2 topic pub /simple_nav_command std_msgs/String "data: stop"

# Go to next waypoint
ros2 topic pub /simple_nav_command std_msgs/String "data: next"
```

## Files

- `astar_navigation_controller.py` - Main navigation code
- `obstacle_spawner.py` - Creates random obstacles
- `goal_setter.py` - Handles commands
- `complete_navigation.launch.py` - Starts everything

## How it works

1. Robot scans area with laser
2. Creates a map of obstacles
3. Uses A* algorithm to find path
4. Follows the path to waypoint
5. Repeats for next waypoint

## Settings

The robot moves in a 6x6 meter box with waypoints at each corner:
- Bottom-left: (-2, -2)
- Bottom-right: (2, -2) 
- Top-right: (2, 2)
- Top-left: (-2, 2)

## Troubleshooting

If robot gets stuck:
- Check if too many obstacles spawned
- Try the "replan" command
- Restart if needed

## Video Demo

https://drive.google.com/file/d/1gJJwqPIQhzi4mFxm4RPEYbWcymg_CO52/view?usp=sharing
