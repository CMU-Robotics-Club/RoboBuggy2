# /usr/bin/env python3
import rospy

# ROS Message Imports
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

import numpy as np
from threading import Lock

from trajectory import Trajectory
from world import World
from controller import Controller
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

    pose: Pose = None
    speed = None

    steer_publisher = None

    def __init__(self, trajectory, controller) -> None:
        self.trajectory = trajectory
        self.controller = controller

        self.lock = Lock()

        rospy.Subscriber("state/pose", PoseStamped, self.tick)
        rospy.Subscriber("state/speed", Float32, self.update_speed)
        self.steer_publisher = rospy.Publisher(
            "buggy/input/steering", Float32, queue_size=1
        )

    def update_speed(self, msg):
        with self.lock:
            self.speed = msg.data

    def tick(self, msg):
        # Received an updated pose from the state estimator
        # Compute the new control output and publish it to the buggy

        with self.lock:
            current_speed = self.speed

        # Get data from message
        pose_gps = Pose.rospose_to_pose(msg.pose)
        pose = World.gps_to_world_pose(pose_gps)

        # Compute control output
        steering_angle = self.controller.compute_control(
            pose, self.trajectory, current_speed
        )

        # Publish control output
        steering_angle_deg = np.rad2deg(steering_angle)
        self.steer_publisher.publish(Float32(steering_angle_deg))
