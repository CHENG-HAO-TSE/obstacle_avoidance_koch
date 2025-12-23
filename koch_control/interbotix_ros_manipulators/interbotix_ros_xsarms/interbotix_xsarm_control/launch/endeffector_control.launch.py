#!/usr/bin/env python3

"""
Launch file for endeffector control.
IMPORTANT: MoveIt must be running first!

Usage:
  ros2 launch interbotix_xsarm_control endeffector_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    robot_name = LaunchConfiguration('robot_name', default='low_cost_robot')
    
    # Get URDF and SRDF files
    xsarm_descriptions_share = FindPackageShare('interbotix_xsarm_descriptions')
    
    urdf_file = PathJoinSubstitution([
        xsarm_descriptions_share,
        'urdf',
        'low_cost_robot.urdf.xacro'
    ])
    
    srdf_file = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
        'srdf',
        'low_cost_robot.srdf'
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='low_cost_robot',
            description='Robot name for namespace'
        ),
        
        # Launch endeffector control node in the robot's namespace
        Node(
            package='interbotix_xsarm_control',
            executable='endeffector_koch',
            name='endeffector_koch',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
        ),
    ])


