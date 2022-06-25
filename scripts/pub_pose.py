#! /usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import ikpy.chain


pkg_dir = get_package_share_directory('ur5_pick_and_place')


ur5e_chain = ikpy.chain.Chain.from_urdf_file(os.path.join(pkg_dir, 'urdf', 'ur5e.urdf'))


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("pub_pose_node")
        # Declare all parameters
        # self.declare_parameter("controller_name", "position_trajectory_controller")
        # self.declare_parameter("wait_sec_between_publish", 6)
        # self.declare_parameter("goal_names", ["pos1", "pos2"])
        # self.declare_parameter("joints")
        # self.declare_parameter("check_starting_point", False)
        # self.declare_parameter("starting_point_limits")

        # ...........Saved configuration................

        pos1_pose = [1.0, 0.0, 1.5]
        pos1_pose_transformed = ur5e_chain.inverse_kinematics(pos1_pose).tolist()[1:7]

        pos2_pose = [0.0, 1.0, 1.5]
        pos2_pose_transformed = ur5e_chain.inverse_kinematics(pos2_pose).tolist()[1:7]

        pos3_pose = [0.0, 0.0, 1.5]
        pos3_pose_transformed = ur5e_chain.inverse_kinematics(pos3_pose).tolist()[1:7]

        # ................................................

        # Read parameters
        controller_name = "joint_trajectory_controller"
        wait_sec_between_publish = 7
        goal_names = ["pos1", "pos2", "pos3"]
        goal_dict = {\
            "pos1": pos1_pose_transformed, \
            "pos2": pos2_pose_transformed, \
            "pos3": pos3_pose_transformed, \

        }
        self.joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.check_starting_point = False
        self.starting_point = {}

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
        # initialize starting point status
        if not self.check_starting_point:
            self.starting_point_ok = True
        else:
            self.starting_point_ok = False

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []
        for name in goal_names:
            # self.declare_parameter(name)
            # goal = self.get_parameter(name).value
            goal = goal_dict[name]
            self.get_logger().info(f"Goal {name} is {goal}")
            if goal is None or len(goal) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal = []
            for value in goal:
                float_goal.append(float(value))
            self.goals.append(float_goal)

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            'Publishing {} goals on topic "{}" every {} s'.format(
                len(goal_names), publish_topic, wait_sec_between_publish
            )
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        if self.starting_point_ok:

            traj = JointTrajectory()
            traj.joint_names = self.joints
            point = JointTrajectoryPoint()
            point.positions = self.goals[self.i]
            point.time_from_start = Duration(sec=4)

            traj.points.append(point)
            self.publisher_.publish(traj)

            self.i += 1
            self.i %= len(self.goals)

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn("Start configuration is not within configured limits!")

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()