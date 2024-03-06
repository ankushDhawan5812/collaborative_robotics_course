#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    # Path to ~/.gazebo/models
    models_path = os.path.expanduser('~/.gazebo/models')
    
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'apriltag_0', 
                '-file', f'{models_path}/Apriltag36_11_00000/model.sdf',
                '-x', '0.0', 
                '-y', '1.0', 
                '-z', '0.0',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0'],
            output='screen'
        )
    ])
