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
from brake_controller import BrakeController
from model_predictive_controller import ModelPredictiveController
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
    brake_controller: BrakeController = None
    lock = None

    steer_publisher = None

    def __init__(self, trajectory, controller, brake_controller) -> None:
        self.trajectory = trajectory
        self.controller = controller
        self.brake_controller = brake_controller

        self.lock = Lock()

        self.msg = None

        rospy.Subscriber("nav/odom", Odometry, self.update_msg)
        self.steer_publisher = rospy.Publisher(
            "buggy/input/steering", Float64, queue_size=1
        )
        self.brake_publisher = rospy.Publisher(
            "buggy/input/brake", Float64, queue_size=1
        )
        self.brake_debug_publisher = rospy.Publisher(
            "auton/debug/brake", Float64, queue_size=1
        )

        self.heading_publisher = rospy.Publisher(
            "auton/debug/heading", Float32, queue_size=1
        )

        self.tick_caller()

    def update_speed(self, msg):
        with self.lock:
            self.speed = msg.data
    
    def update_msg(self, msg):
        with self.lock:
            self.msg = msg
        
    def tick_caller(self):
        while(self.msg == None):
            rospy.sleep(0.001)
        while (True):
            self.tick()
            rospy.sleep(0.001)

    def tick(self):
        # Received an updated pose from the state estimator
        # Compute the new control output and publish it to the buggy
        with self.lock:
            msg = self.msg
        current_rospose = msg.pose.pose

        # Check if the pose covariance is a sane value. Otherwise ignore the message
        if msg.pose.covariance[0] ** 2 + msg.pose.covariance[7] ** 2 > 1**2:
            # Covariance larger than one meter. We definitely can't trust the pose
            return

        current_speed = np.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )

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
        brake_cmd = self.brake_controller.compute_braking(current_speed, steering_angle_deg)
        self.brake_debug_publisher.publish(Float64(brake_cmd))
        self.brake_publisher.publish(Float64(0)) # No braking for now, just look at debug data

        # Publish debug data
        self.heading_publisher.publish(Float32(pose.theta))


if __name__ == "__main__":
    rospy.init_node("auton_system")
    auton_system = AutonSystem(
        Trajectory("/rb_ws/src/buggy/paths/quartermiletrack.json"),
        # PurePursuitController(),
        # StanleyController(),
        ModelPredictiveController(),
        BrakeController()
    )
    while not rospy.is_shutdown():
        rospy.spin()
