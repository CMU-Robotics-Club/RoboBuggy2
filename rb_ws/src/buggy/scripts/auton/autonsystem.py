#!/usr/bin/env python3

import rospy
import sys

# ROS Message Imports
from std_msgs.msg import Float32, Float64, Bool
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
# from model_predictive_interpolation import ModelPredictiveController
from path_planner import PathPlanner
from pose import Pose
import argparse
import copy
import cProfile

import time

class AutonSystem:
    """
    Top-level class for the RoboBuggy autonomous system

    On every tick, this class will read the current state of the buggy,
    compare it to the reference trajectory, and use a given controller
    to compute the desired control output.

    On every 10th tick, this class will generate a new reference trajectory
    based on the updated position of the other buggy.
    """

    global_trajectory: Trajectory = None
    local_controller: Controller = None
    nominal_controller: Controller = None
    brake_controller: BrakeController = None
    lock = None

    steer_publisher = None
    ticks = 0

    def __init__(self, global_trajectory, local_controller, nominal_controller, brake_controller, self_name, other_name) -> None:
        self.global_trajectory = global_trajectory

        # local trajectory is initialized as global trajectory. If there is no other buggy,
        # the local trajectory is never updated.

        self.has_other_buggy = not other_name is None
        self.cur_traj = global_trajectory
        self.nominal_controller = nominal_controller
        self.local_controller = local_controller
        self.brake_controller = brake_controller

        self.path_planner = PathPlanner(global_trajectory)
        self.other_steering = 0

        self.lock = Lock()
        self.ticks = 0
        self.self_odom_msg = None
        self.other_odom_msg = None
        
        rospy.Subscriber(self_name + "/nav/odom", Odometry, self.update_self_odom)
        if self.has_other_buggy:
            rospy.Subscriber(other_name + "/nav/odom", Odometry, self.update_other_odom)
            self.other_steer_subscriber = rospy.Subscriber(
                other_name + "/input/steering", Float64, self.update_other_steering_angle
            )


        self.covariance_warning_publisher = rospy.Publisher(
            self_name + "/debug/is_high_covariance", Bool, queue_size=1
        )
        self.steer_publisher = rospy.Publisher(
            self_name + "/input/steering", Float64, queue_size=1
        )
        self.brake_publisher = rospy.Publisher(
            self_name + "/input/brake", Float64, queue_size=1
        )
        self.brake_debug_publisher = rospy.Publisher(
            self_name + "/auton/debug/brake", Float64, queue_size=1
        )
        self.heading_publisher = rospy.Publisher(
            self_name + "/auton/debug/heading", Float32, queue_size=1
        )
        self.distance_publisher = rospy.Publisher(
            self_name + "/auton/debug/distance", Float64, queue_size=1
        )


        self.auton_rate = 100
        self.rosrate = rospy.Rate(self.auton_rate)
        self.tick_caller()

    def update_self_odom(self, msg):
        with self.lock:
            self.self_odom_msg = msg

    def update_other_odom(self, msg):
        with self.lock:
            self.other_odom_msg = msg
    
    def update_other_steering_angle(self, msg):
        with self.lock:
            self.other_steering = msg.data

    def x(self):
        self.planner_tick()

    def tick_caller(self):
        while ((not rospy.is_shutdown()) and
            (self.self_odom_msg == None or 
            (self.has_other_buggy and self.other_odom_msg == None))): # with no message, we wait
            rospy.sleep(0.001)
        
        # wait for covariance matrix to be better
        while ((not rospy.is_shutdown()) and
               (self.self_odom_msg.pose.covariance[0] ** 2 + self.self_odom_msg.pose.covariance[7] ** 2 > 1**2)):
            # Covariance larger than one meter. We definitely can't trust the pose
            rospy.sleep(0.001)
            print("Waiting for Covariance to be better: ",  rospy.get_rostime())
        print("done checking covariance")

        # initialize global trajectory index

        with self.lock:
            e, _ = self.get_world_pose_and_speed(self.self_odom_msg)

        while (not rospy.is_shutdown()): # start the actual control loop
            # run the planner every 10 ticks
            # thr main cycle runs at 100hz, the planner runs at 10hz, but looks 1 second ahead
            if not self.other_odom_msg is None and self.ticks % 5 == 0:         
                # for debugging obstacle avoidance
                with self.lock:
                    self_pose, self_speed = self.get_world_pose_and_speed(self.self_odom_msg)
                    other_pose, other_speed = self.get_world_pose_and_speed(self.other_odom_msg)
                    distance = (self_pose.x - other_pose.x) ** 2 + (self_pose.y - other_pose.y) ** 2
                    distance = np.sqrt(distance)
                    self.distance_publisher.publish(Float64(distance))

                cProfile.runctx('self.x()', globals(), locals(), sort="cumtime")
                # reset current index of local controller, since local trajectory is updated
            else:
                self.local_controller_tick()

            self.ticks += 1
            self.rosrate.sleep() 

        
    def get_world_pose_and_speed(self, msg):
        current_rospose = msg.pose.pose
        # Check if the pose covariance is a sane value. Publish a warning if insane
        if msg.pose.covariance[0] ** 2 + msg.pose.covariance[7] ** 2 > 1**2:
            # Covariance larger than one meter. We definitely can't trust the pose
            self.covariance_warning_publisher.publish(Bool(True))
        else:
            self.covariance_warning_publisher.publish(Bool(False))
        current_speed = np.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )

        # Get data from message
        pose_gps = Pose.rospose_to_pose(current_rospose)
        return World.gps_to_world_pose(pose_gps), current_speed
    
    def local_controller_tick(self):
        with self.lock:
            self_pose, self_speed = self.get_world_pose_and_speed(self.self_odom_msg)

        # Compute control output
        steering_angle = self.local_controller.compute_control(
            self_pose, self.cur_traj, self_speed)
        steering_angle_deg = np.rad2deg(steering_angle)
        self.steer_publisher.publish(Float64(steering_angle_deg))

    def nominal_controller_tick(self):
        with self.lock:
            self_pose, self_speed = self.get_world_pose_and_speed(self.self_odom_msg)

        # Compute control output from global static trajectory
        steering_angle = self.nominal_controller.compute_control(
            self_pose, self.global_trajectory, self_speed)
        steering_angle_deg = np.rad2deg(steering_angle)
        return float(steering_angle_deg)

    def planner_tick(self):
        with self.lock:
            self_pose, self_speed = self.get_world_pose_and_speed(self.self_odom_msg)
            other_pose, other_speed = self.get_world_pose_and_speed(self.other_odom_msg)
        # update local trajectory via path planner
        self.cur_traj = self.path_planner.compute_traj(self_pose, 
                                            other_pose,
                                            self_speed, 
                                            other_speed, 
                                            0, 
                                            self.other_steering)
if __name__ == "__main__":
    rospy.init_node("auton_system")
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", type=str, help="set controller type", required=True)
    parser.add_argument("--dist", type=float,
        help="start buggy at meters distance along the path", required=True)
    parser.add_argument("--traj", type=str, 
        help="path to the trajectory file, relative to /rb_ws/src/buggy/paths/", 
        required=True)
    parser.add_argument("--self_name", type=str, help="name of ego-buggy", 
        required=True)
    parser.add_argument("--other_name", type=str, help="name of other buggy, if left unspecified, the autonsystem assumes it is the only buggy on the course",         
        required=False)
    
    args, _ = parser.parse_known_args()
    ctrl = args.controller
    start_dist = args.dist
    traj = args.traj
    self_name = args.self_name
    other_name = args.other_name

    print("\n\nStarting Controller: " + str(ctrl) + "\n\n")
    print("\n\nUsing path: /rb_ws/src/buggy/paths/" + str(traj) + "\n\n")
    print("\n\nStarting at distance: " + str(start_dist) + "\n\n")

    trajectory = Trajectory(json_filepath="/rb_ws/src/buggy/paths/" + traj)

    # calculate starting index
    start_index = trajectory.get_index_from_distance(start_dist)

    # Add Controllers Here
    local_ctrller = None
    nominal_ctrller = None
    if (ctrl == "stanley"):
        local_ctrller = StanleyController(self_name + "_local", start_index)
        nominal_ctrller = StanleyController(self_name + "_nominal", start_index)
    elif (ctrl == "pure_pursuit"):
        local_ctrller = PurePursuitController(self_name + "_local", start_index)
        nominal_ctrller = PurePursuitController(self_name + "_nominal", start_index)
    elif (ctrl == "mpc"):
        print(start_index)
        print(self_name + "local")
        local_ctrller = ModelPredictiveController(self_name + "_local")
        nominal_ctrller = ModelPredictiveController(self_name + "_nominal")
    if (local_ctrller == None):
        raise Exception("Invalid Controller Argument")
    
    auton_system = AutonSystem(
        trajectory,
        local_ctrller,
        nominal_ctrller,
        BrakeController(),
        self_name,
        other_name
    )

    while not rospy.is_shutdown():
        rospy.spin()
