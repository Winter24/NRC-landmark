#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    args = [
        DeclareLaunchArgument('min_intensity',        default_value='50.0'),
        DeclareLaunchArgument('angle_cluster_thresh', default_value='0.055'),
        DeclareLaunchArgument('range_min',            default_value='0.2'),
        DeclareLaunchArgument('range_max',            default_value='10.0'),
    ]

    node = Node(
        package='landmark_detection',
        executable='landmark_detection',
        name='landmark_detection',
        output='screen',
        parameters=[
            {'min_intensity':        LaunchConfiguration('min_intensity')},
            {'angle_cluster_thresh': LaunchConfiguration('angle_cluster_thresh')},
            {'range_min':            LaunchConfiguration('range_min')},
            {'range_max':            LaunchConfiguration('range_max')},
        ],
        remappings=[
            ('/scan', '/sensors/lidar_pp_mnl'),
            ('/reflector_poses', '/reflector_poses'),
        ],
    )

    return LaunchDescription(args + [node])
