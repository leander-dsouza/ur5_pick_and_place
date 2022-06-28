#! /usr/bin/env python3
"""
Program to generate boilerplate code for ROS2 scripts.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class ROS2BoilerPlate(Node):
    """
    ROS2 Boilerplate Class
    """

    def __init__(self):
        super().__init__('ros2_boilerplate_node')

        self.update_rate = 50
        self.time_period = 1./self.update_rate

        # Subscribers
        self.create_subscription(String, '/chatter', self.chatter_callback, \
            qos_profile_sensor_data)

        # Publishers
        self.pub = self.create_publisher(String, "/chatter", \
            qos_profile_sensor_data)

        # Timers
        self.create_timer(self.time_period, self.chatter_update)

    def chatter_callback(self, msg):
        """
        Callback function for the chatter topic.
        """
        self.get_logger().info('Subscribing: %s' % msg.data)

    def chatter_update(self):
        """
        Function to update the chatter update
        """
        chatter_msg = String()
        chatter_msg.data = 'Hello World: {}'.format(Node.get_clock(self).now().to_msg().sec)
        self.pub.publish(chatter_msg)

        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % chatter_msg.data)


def main(args=None):
    """
    Main Function
    """
    rclpy.init(args=args)

    ros2_boilerplate = ROS2BoilerPlate()
    rclpy.spin(ros2_boilerplate)

    ros2_boilerplate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()