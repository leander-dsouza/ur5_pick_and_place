#! /usr/bin/env python3
"""
Main Launch file
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch Function
    """

    return LaunchDescription([

        Node(
            package='ur5_pick_and_place',
            executable='pub_pose.py',
            name='pub_pose_node',
            output='screen',
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["7", "0", "0", "0", "0"," 0", "base_link", "camera_link"],
            output="screen"
        ),


    ])
