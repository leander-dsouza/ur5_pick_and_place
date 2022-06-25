#! /usr/bin/env python3
"""
Main Launch file
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

rs_pkg_dir = get_package_share_directory('realsense2_camera')
ur_pkg_dir = get_package_share_directory('ur_bringup')

def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    enable_pcl = False
    robot_ip = "192.168.56.1"

    # ...............................................................


    return LaunchDescription([

        DeclareLaunchArgument("enable_pcl", \
            default_value=str(enable_pcl), \
                description="Enable PointCloud"),

        DeclareLaunchArgument("robot_ip", \
            default_value=robot_ip, \
                description="UR5e robot IP Address"),

        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [rs_pkg_dir, '/launch', '/rs_launch.py']),
            launch_arguments={
                'pointcloud.enable': LaunchConfiguration('enable_pcl'),
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ur_pkg_dir, '/launch', '/ur5e_launch.py']),
            launch_arguments={
                'robot_ip': LaunchConfiguration('robot_ip'),
                }.items(),
        ),

    ])
