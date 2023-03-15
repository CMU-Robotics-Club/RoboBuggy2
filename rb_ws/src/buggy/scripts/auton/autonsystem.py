#!/usr/bin/env python3

import rospy

# ROS Message Imports
from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Odometry

import numpy as np
from threading import Lock

from trajectory import Trajectory
from world import World
from controller import Controller
from pure_pursuit_controller import PurePursuitController
from stanley_controller import StanleyController
from pose import Pose


class AutonSystem:
    """
    Top-level class for the RoboBuggy autonomous system

    On every tick, this class will read the current state of the buggy,
    compare it to the reference trajectory, and use a given controller
    to compute the desired control output.
    """

    trajectory: Trajectory = None
    controller: Controller = None
    lock = None

    steer_publisher = None

    def __init__(self, trajectory, controller) -> None:
        self.trajectory = trajectory
        self.controller = controller

        self.lock = Lock()

        rospy.Subscriber("nav/odom", Odometry, self.tick)
        self.steer_publisher = rospy.Publisher(
            "buggy/input/steering", Float64, queue_size=1
        )
        self.brake_publisher = rospy.Publisher(
            "buggy/input/brake", Float64, queue_size=1
        )

        self.heading_publisher = rospy.Publisher(
            "auton/debug/heading", Float32, queue_size=1
        )

    def update_speed(self, msg):
        with self.lock:
            self.speed = msg.data

    def tick(self, msg):
        # Received an updated pose from the state estimator
        # Compute the new control output and publish it to the buggy

        current_speed = np.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        current_rospose = msg.pose.pose

        # Get data from message
        pose_gps = Pose.rospose_to_pose(current_rospose)
        pose = World.gps_to_world_pose(pose_gps)

        # Compute control output
        steering_angle = self.controller.compute_control(
            pose, self.trajectory, current_speed
        )

        # Publish control output
        steering_angle_deg = np.rad2deg(steering_angle)
        self.steer_publisher.publish(Float64(steering_angle_deg))
        self.brake_publisher.publish(Float64(0))

        # Publish debug data
        self.heading_publisher.publish(Float32(pose.theta))


if __name__ == "__main__":
    rospy.init_node("auton_system")
    auton_system = AutonSystem(
        Trajectory("/rb_ws/src/buggy/paths/quartermiletrack.json"),
        # PurePursuitController(),
        StanleyController(),
    )
    while not rospy.is_shutdown():
        rospy.spin()
