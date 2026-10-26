#!/usr/bin/env python3
# The Python NMPC (quad_nmpc_ros/nmpc_node.py), installed as `nmpc_node_py` so it
# does not clash with the C++ `nmpc_node`. It sends motors only, so fly it with
# the motor world:
#   ros2 run quad_nmpc_ros_cpp nmpc_node_py
#   ros2 run quad_nmpc_ros_cpp sim_node --ros-args -p acro_mode:=false
from quad_nmpc_ros.nmpc_node import main

main()
