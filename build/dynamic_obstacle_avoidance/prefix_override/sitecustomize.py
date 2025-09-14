import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kits/turtlebot3_ws/install/dynamic_obstacle_avoidance'
