import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pari/GestureNav/ros2_ws/install/gesture_drone_control'
