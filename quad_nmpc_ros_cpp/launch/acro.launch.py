"""C++ NMPC through AcroMode: the 3 nodes of the README's "terminal 1/2/3" in one command.

  ros2 launch quad_nmpc_ros_cpp acro.launch.py
  ros2 launch quad_nmpc_ros_cpp acro.launch.py duration:=60.0 log_file:=/tmp/log.csv

nmpc_node -> trpy_converter_node -> AcroMode (in sim_node). The sim starts 4 s after the
controller side, so its first /odom is already answered. When nmpc_node ends (duration > 0)
the whole launch shuts down.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    duration = LaunchConfiguration('duration')
    log_file = LaunchConfiguration('log_file')
    sim_delay = LaunchConfiguration('sim_delay')

    nmpc = Node(package='quad_nmpc_ros_cpp', executable='nmpc_node', output='screen',
                parameters=[{'duration': duration, 'log_file': log_file}])
    converter = Node(package='quad_nmpc_ros_cpp', executable='trpy_converter_node',
                     output='screen')
    sim = Node(package='quad_nmpc_ros_cpp', executable='sim_node', output='screen',
               parameters=[{'acro_mode': True}])

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='0.0',
                              description='nmpc_node run time [s], 0 = forever'),
        DeclareLaunchArgument('log_file', default_value='',
                              description='nmpc_node CSV log path, empty = no log'),
        DeclareLaunchArgument('sim_delay', default_value='4.0',
                              description='wait before starting sim_node [s]'),
        nmpc,
        converter,
        TimerAction(period=sim_delay, actions=[sim]),
        RegisterEventHandler(OnProcessExit(target_action=nmpc,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
