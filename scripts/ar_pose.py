#! /usr/bin/env python3
"""
Program to recieve AR poses.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from .tf_utils import do_transform_pose

class ARPose(Node):
    """
    ROS2 Boilerplate Class
    """

    def __init__(self):
        super().__init__('ar_pose_node')

        self.update_rate = 50
        self.time_period = 1./self.update_rate

        self.ar_pose = Pose()

        # Create a tf2 buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Subscribers
        self.create_subscription(PoseArray, '/aruco_poses', self.aruco_callback, \
            qos_profile_sensor_data)

        # Timers
        self.create_timer(self.time_period, self.aruco_pose_update)

    def aruco_callback(self, msg):
        """
        Callback function for the aruco_poses topic.
        """
        self.ar_pose = msg.poses[0]

    def aruco_pose_update(self):
        """
        Function to update the chatter update
        """
        # Transform from two frames

        try:
            transform = self.buffer.lookup_transform('base_link', 'camera_color_optical_frame', Time(nanoseconds=0))
            print(transform)

            pose_transformed = do_transform_pose(self.ar_pose, transform)

        except:
            print("Frame not found")



def main(args=None):
    """
    Main Function
    """
    rclpy.init(args=args)

    ar_pose = ARPose()
    rclpy.spin(ar_pose)

    ar_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()