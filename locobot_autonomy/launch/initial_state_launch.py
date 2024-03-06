#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Launching other launch files
    # launch_file1 = os.path.join(os.path.dirname(__file__), '/home/ankushdhawan/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_sim/launch/xslocobot_gz_classic.launch.py')
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(launch_file1),
    #     launch_arguments={'robot_model': 'locobot_wx250s', 'use_sim_time': 'true'}.items()
    # ))

    # Launch moveit launch file
    launch_file2 = os.path.join(os.path.dirname(__file__), '/home/ankushdhawan/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_moveit/launch/xslocobot_moveit.launch.py')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file2),
        #launch_arguments={'robot_model': 'locobot_wx250s', 'use_sim_time': 'true', 'hardware_type': 'gz_classic', 'use_lidar': 'true'}.items()
        launch_arguments={'robot_model': 'locobot_wx250s', 'use_sim_time': 'true', 'hardware_type': 'gz_classic', 'use_lidar': 'true'}.items()

    ))

    # Spawn cubes
    launch_file3 = os.path.join(os.path.dirname(__file__), 'spawn_cube_launch.py')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file3)
    ))

    # Launching move base action server
    ld.add_action(Node(
        package='locobot_autonomy',
        executable='move_locobot_base_action_server.py',
        name='movebase_action_server',
        output='screen',
        #parameters=[{'param_name': 'param_value'}]  # Optional parameters
    ))

    # Launching perception service
    ld.add_action(Node(
        package='locobot_autonomy',
        executable='visual_block_perception_srv',
        name='Matching_Pix_to_Ptcld',
        output='screen',
        # parameters=[{'param_name': 'param_value'}]  # Optional parameters
    ))

    # Launching moveit arm action server 
    ld.add_action(Node(
        package='locobot_autonomy',
        executable='move_arm_action_srv',
        name='action_server_node',
        output='screen',
        # parameters=[{'param_name': 'param_value'}]  # Optional parameters
    ))

    # Launching moveit gripper action server 
    ld.add_action(Node(
        package='locobot_autonomy',
        executable='move_gripper_action_server.py',
        name='movebase_gripper_action_server',
        output='screen',
        # parameters=[{'param_name': 'param_value'}]  # Optional parameters
    ))

    return ld
