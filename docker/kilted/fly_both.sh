#!/bin/bash
# Fly the 60 s circle once per pipeline and drop each nmpc_node flight log in /results:
#   /results/acromode/log.csv       nmpc_node -> converter -> AcroMode
#   /results/motor-direct/log.csv   nmpc_node -> sim_node sets the 4 motors
# The controller side starts first, so the very first /odom is already answered.
source /opt/ros/kilted/setup.bash
source /ws/install/setup.bash

fly() {  # fly <name> <acro_mode true|false>
  local out=/results/$1; mkdir -p "$out"
  ros2 run quad_nmpc_ros_cpp nmpc_node --ros-args -p duration:=60.0 \
      -p log_file:="$out/log.csv" > "$out/nmpc_node.out" 2>&1 &
  local nmpc=$! conv=
  if [ "$2" = true ]; then
    ros2 run quad_nmpc_ros_cpp trpy_converter_node > "$out/converter.out" 2>&1 &
    conv=$!
  fi
  sleep 4
  ros2 run quad_nmpc_ros_cpp sim_node --ros-args -p acro_mode:="$2" -p duration:=63.0 \
      > "$out/sim_node.out" 2>&1 &
  local sim=$!
  wait $nmpc
  sleep 3
  kill -INT $sim $conv 2>/dev/null; sleep 2; kill -9 $sim $conv 2>/dev/null
  echo "$1: $(($(wc -l < "$out/log.csv") - 1)) rows, $(grep -c 'solver status' "$out/nmpc_node.out") bad solves"
}

fly acromode true
fly motor-direct false
