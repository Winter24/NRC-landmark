#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Khai báo các launch argument
    args = [
        DeclareLaunchArgument('min_intensity',           default_value='200.0'),    #Ngưỡng cường độ phản xạ 
        DeclareLaunchArgument('range_min',               default_value='0.2'),      
        DeclareLaunchArgument('range_max',               default_value='10.0'),     
        DeclareLaunchArgument('angle_cluster_thresh',    default_value='0.01'),     #Ngưỡng sai khác góc giữa hai điểm liên tiếp khi thực hiện clustering
        DeclareLaunchArgument('dist_cluster_thresh',     default_value='0.05'),     #Ngưỡng khoảng cách Euclid giữa hai điểm liên tiếp khi clustering

        DeclareLaunchArgument('ransac_thresh',           default_value='0.01'),     #Ngưỡng dể chấp nhận inliner
        DeclareLaunchArgument('min_cluster_size',        default_value='3'),        #Số điểm tối thiểu để một tập điểm được coi là cụm hợp lệ
        DeclareLaunchArgument('line_ransac_iters',       default_value='100'),      #Số vòng lặp tối đa cho quá trình RANSAC
        
        DeclareLaunchArgument('corner_angle_tol',        default_value='0.5'),      #Sai số cho phép so với góc vuông (90°) khi so sánh hai line để phát hiện corner
        DeclareLaunchArgument('max_cluster_centroid_dist', default_value='2.0'),    #Khoảng cách tối đa giữa hai centroid của hai cụm để chúng được coi là “liền kề” và có thể ghép đôi để tìm corner.
    ]

    # Node landmark detector
    node = Node(
        package='landmark_detection',                # tên package của bạn
        executable='landmark_detection_line_corner',     # tên executable sau khi build
        name='landmark_detection_line_corner',
        output='screen',
        parameters=[
            {'min_intensity': LaunchConfiguration('min_intensity')},
            {'range_min':       LaunchConfiguration('range_min')},
            {'range_max':       LaunchConfiguration('range_max')},
            {'angle_cluster_thresh':    LaunchConfiguration('angle_cluster_thresh')},
            {'dist_cluster_thresh':     LaunchConfiguration('dist_cluster_thresh')},
            {'ransac_thresh':           LaunchConfiguration('ransac_thresh')},
            {'min_cluster_size':        LaunchConfiguration('min_cluster_size')},
            {'line_ransac_iters':       LaunchConfiguration('line_ransac_iters')},
            {'corner_angle_tol':        LaunchConfiguration('corner_angle_tol')},
            {'max_cluster_centroid_dist': LaunchConfiguration('max_cluster_centroid_dist')},
        ],
        remappings=[
            ('/scan',           '/sensors/lidar_pp_mnl'),
            ('/line_landmarks', '/line_landmarks'),
            ('/corner_landmarks', '/corner_landmarks'),
            ('/cluster_markers','/cluster_markers'),
        ],
    )

    return LaunchDescription(args + [node])
