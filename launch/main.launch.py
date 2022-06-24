#! /usr/bin/env python3
"""
Main Launch file
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

#pkg_dir = get_package_share_directory('realsense2_camera')

def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    enable_pcl = False

    # ...............................................................


    return LaunchDescription([

        DeclareLaunchArgument("enable_pcl", \
            default_value=str(enable_pcl), \
                description="Enable PointCloud"),

        Node(
            package='ur5_pick_and_place',
            executable='pub_pose.py',
            name='pub_pose_node',
            output='screen',
        ),

        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            output='screen',
        ),


        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource( \
        #         [pkg_dir, '/launch', '/rs_launch.py']),
        #     launch_arguments={
        #         ' pointcloud.enable': LaunchConfiguration('enable_pcl'),
        #         }.items(),
        # ),

    ])
