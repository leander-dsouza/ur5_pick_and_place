#! /usr/bin/env python3
"""
Program to recieve AR poses.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray


class ARPose(Node):
    """
    ROS2 Boilerplate Class
    """

    def __init__(self):
        super().__init__('ar_pose_node')

        self.update_rate = 50
        self.time_period = 1./self.update_rate

        # Subscribers
        self.create_subscription(PoseArray, '/aruco_poses', self.aruco_callback, \
            qos_profile_sensor_data)

        # Timers
        self.create_timer(self.time_period, self.chatter_update)

    def aruco_callback(self, msg):
        """
        Callback function for the aruco_poses topic.
        """
        self.get_logger().info(msg)

    def chatter_update(self):
        """
        Function to update the chatter update
        """
        pass



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