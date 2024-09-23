#!/usr/bin/env python3

import argparse
from threading import Lock

import threading
import rospy

# ROS Message Imports
from std_msgs.msg import Float32, Float64, Bool
from nav_msgs.msg import Odometry
from buggy.msg import TrajectoryMsg

import numpy as np

from trajectory import Trajectory
from world import World
from controller import Controller
from stanley_controller import StanleyController
from model_predictive_controller import ModelPredictiveController
from path_planner import PathPlanner
from pose import Pose

import navpy

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
    lock = None
    steer_publisher = None
    ticks = 0

    def __init__(self,
            global_trajectory,
            local_controller,
            self_name,
            other_name,
            curb_traj,
            profile) -> None:


        self.global_trajectory = global_trajectory

        # local trajectory is initialized as global trajectory. If there is no other buggy,
        # the local trajectory is never updated.

        self.has_other_buggy = not (other_name is None)
        self.cur_traj = global_trajectory
        self.local_controller = local_controller

        left_curb = curb_traj
        self.path_planner = PathPlanner(global_trajectory, left_curb)
        self.other_steering = 0

        self.lock = Lock()
        self.ticks = 0
        self.self_odom_msg = None
        self.gps_odom_msg = None
        self.other_odom_msg = None
        self.use_gps_pos = False

        #TODO: DOUBLE CONVERTING HERE, NOT A GOOD IDEA
        rospy.Subscriber("/ekf/odometry_earth", Odometry, self.change_utm_latlon)
        self.latlonodom = rospy.Publisher(self_name + "/nav/odom", Odometry, queue_size=1)

        rospy.Subscriber(self_name + "/nav/odom", Odometry, self.update_self_odom)
        rospy.Subscriber(self_name + "/gnss1/odom", Odometry, self.update_self_odom_backup)
        rospy.Subscriber(self_name + "/nav/traj", TrajectoryMsg, self.update_traj)

        # to report if the filter position has separated (so we need to use the antenna position)
        rospy.Subscriber(self_name + "/debug/filter_gps_seperation_status", Bool, self.update_use_gps)

        if self.has_other_buggy:
            rospy.Subscriber(other_name + "/nav/odom", Odometry, self.update_other_odom)
            self.other_steer_subscriber = rospy.Subscriber(
                other_name + "/buggy/input/steering", Float64, self.update_other_steering_angle
            )

        self.init_check_publisher = rospy.Publisher(
            self_name + "/debug/init_safety_check", Bool, queue_size=1
        )
        self.steer_publisher = rospy.Publisher(
            self_name + "/buggy/input/steering", Float64, queue_size=1
        )
        self.heading_publisher = rospy.Publisher(
            self_name + "/auton/debug/heading", Float32, queue_size=1
        )
        self.distance_publisher = rospy.Publisher(
            self_name + "/auton/debug/distance", Float64, queue_size=1
        )

        self.controller_rate = 100
        self.rosrate_controller = rospy.Rate(self.controller_rate)

        self.planner_rate = 10
        self.rosrate_planner = rospy.Rate(self.planner_rate)

        self.profile = profile
        self.tick_caller()

    def change_utm_latlon(self, msg):
        new_msg = msg
        lat, lon, _ = navpy.ecef2lla([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        new_msg.pose.pose.position.x, new_msg.pose.pose.position.y = lon, lat
        self.latlonodom.publish(new_msg)


    # functions to read data from ROS nodes
    def update_use_gps(self, msg):
        with self.lock:
            self.use_gps_pos = msg.data

    def update_self_odom_backup(self, msg):
        with self.lock:
            self.gps_odom_msg = msg

    def update_self_odom(self, msg):
        with self.lock:
            self.self_odom_msg = msg

    def update_other_odom(self, msg):
        with self.lock:
            self.other_odom_msg = msg

    def update_other_steering_angle(self, msg):
        with self.lock:
            self.other_steering = msg.data

    def update_traj(self, msg):
        with self.lock:
            self.cur_traj, self.local_controller.current_traj_index = Trajectory.unpack(msg)



    def init_check(self):
        """
        Checks if it's safe to switch the buggy into autonomous driving mode.
        Specifically, it checks:
            if we can recieve odometry messages from the buggy
            if the covariance is acceptable (less than 1 meter)
            if the buggy thinks it is facing in the correct direction wrt the local trajectory (not 180 degrees flipped)

        Returns:
           A boolean describing the status of the buggy (safe for auton or unsafe for auton)
        """
        # TODO: should we check if we're recieving messages from NAND?
        if (self.self_odom_msg == None):
            rospy.logwarn("WARNING: no available position estimate")
            return False

        if (self.self_odom_msg.pose.covariance[0] ** 2 + self.self_odom_msg.pose.covariance[7] ** 2 > 1**2):
            rospy.logwarn("checking position estimate certainty")
            return False

        # waits until covariance is acceptable to check heading
        with self.lock:
            self_pose = self.get_world_pose(self.self_odom_msg)
            current_heading = self_pose.theta
            closest_heading = self.cur_traj.get_heading_by_index(trajectory.get_closest_index_on_path(self_pose.x, self_pose.y))
        rospy.loginfo("current heading: " + str(np.rad2deg(current_heading)))
        self.heading_publisher.publish(Float32(np.rad2deg(current_heading)))

        # headings are originally between -pi and pi
        # if they are negative, convert them to be between 0 and pi
        if current_heading < 0:
            current_heading = 2*np.pi + current_heading

        if closest_heading < 0:
            closest_heading = 2*np.pi + closest_heading

        if (abs(current_heading - closest_heading) >= np.pi/2):
            rospy.logwarn("WARNING: INCORRECT HEADING! restart stack. Current heading [-180, 180]: " + str(np.rad2deg(self_pose.theta)))
            return False

        return True

    def tick_caller(self):
        """
        The main scheduler - starts threads for the pathplanner and the controller.
        """
        rospy.loginfo("start checking initialization status")
        while ((not rospy.is_shutdown()) and not self.init_check()):
            self.init_check_publisher.publish(False)
            rospy.sleep(0.001)
        rospy.loginfo("done checking initialization status")
        self.init_check_publisher.publish(True)
        # initialize global trajectory index

        t_planner = threading.Thread(target=self.planner_thread)
        t_controller = threading.Thread(target=self.local_controller_thread)

        # starting processes
        # See LOOKAHEAD_TIME in path_planner.py for the horizon of the
        # planner. Make sure it is significantly (at least 2x) longer
        # than 1 period of the planner when you change the planner frequency.


        t_controller.start() #Main Cycles runs at 100hz
        if self.has_other_buggy:
            t_planner.start() #Planner runs every 10 hz

        t_controller.join()
        if self.has_other_buggy:
            t_planner.join()

    def get_world_pose(self, msg):
        #TODO: this should be redundant - converting rospose to pose should automatically handle world conversions
        current_rospose = msg.pose.pose
        # Get data from message
        pose_gps = Pose.rospose_to_pose(current_rospose)
        return World.gps_to_world_pose(pose_gps)

    def local_controller_thread(self):
        while (not rospy.is_shutdown()):
            self.local_controller_tick()
            self.rosrate_controller.sleep()

    def local_controller_tick(self):
        if not self.use_gps_pos:
            with self.lock:
                state_msg = self.self_odom_msg
        else:
            with self.lock:
                state_msg = self.gps_odom_msg

        # For viz and debugging
        pose = self.get_world_pose(state_msg)
        self.heading_publisher.publish(Float32(np.rad2deg(pose.theta)))

        # Compute control output
        steering_angle = self.local_controller.compute_control(
            state_msg, self.cur_traj)
        steering_angle_deg = np.rad2deg(steering_angle)
        self.steer_publisher.publish(Float64(steering_angle_deg))


    def planner_thread(self):
        while (not rospy.is_shutdown()):
            self.rosrate_planner.sleep()
            if not self.other_odom_msg is None:
                with self.lock:
                    self_pose = self.get_world_pose(self.self_odom_msg)
                    other_pose = self.get_world_pose(self.other_odom_msg)

                distance = (self_pose.x - other_pose.x) ** 2 + (self_pose.y - other_pose.y) ** 2
                distance = np.sqrt(distance)
                self.distance_publisher.publish(Float64(distance))

                self.planner_tick()

    def planner_tick(self):
        if not self.use_gps_pos:
            with self.lock:
                self_pose = self.get_world_pose(self.self_odom_msg)
        else:
            with self.lock:
                self_pose = self.get_world_pose(self.gps_odom_msg)

        with self.lock:
            other_pose = self.get_world_pose(self.other_odom_msg)

        # update local trajectory via path planner
        self.path_planner.compute_traj(self_pose, other_pose)

def init_parser ():
    """
        Returns a parser to read launch file arguments to AutonSystem.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller",
        type=str,
        help="set controller type",
        required=True)

    parser.add_argument(
        "--dist",
        type=float,
        help="start buggy at meters distance along the path",
        required=True)

    parser.add_argument(
        "--traj",
        type=str,
        help="path to the trajectory file, relative to /rb_ws/src/buggy/paths/",
        required=True)

    parser.add_argument(
        "--self_name",
        type=str,
        help="name of ego-buggy",
        required=True)

    parser.add_argument(
        "--left_curb",
        type=str,
        help="Path of curb data, relative to /rb_ws/src/buggy/paths/",
        default=""
,        required=False)

    parser.add_argument(
        "--other_name",
        type=str,
        help="name of other buggy, if left unspecified, the autonsystem assumes it is the only buggy on the course",
        required=False)

    parser.add_argument(
        "--profile",
        action='store_true',
        help="turn on profiling for the path planner")
    return parser

if __name__ == "__main__":
    rospy.init_node("auton_system")
    parser = init_parser()

    args, _ = parser.parse_known_args()
    ctrl = args.controller
    start_dist = args.dist
    traj = args.traj
    self_name = args.self_name
    other_name = args.other_name
    profile = args.profile
    left_curb_file = args.left_curb

    rospy.loginfo("\n\nStarting Controller: " + str(ctrl) + "\n\n")
    rospy.loginfo("\n\nUsing path: /rb_ws/src/buggy/paths/" + str(traj) + "\n\n")
    rospy.loginfo("\n\nStarting at distance: " + str(start_dist) + "\n\n")

    trajectory = Trajectory(json_filepath="/rb_ws/src/buggy/paths/" + traj)
    left_curb = None
    if left_curb_file != "":
        left_curb = Trajectory(json_filepath="/rb_ws/src/buggy/paths/" + left_curb_file)

    # calculate starting index
    start_index = trajectory.get_index_from_distance(start_dist)

    # Add Controllers Here
    local_ctrller = None
    if (ctrl == "stanley"):
        local_ctrller = StanleyController(
            self_name,
            start_index=start_index)
    elif (ctrl == "mpc"):
        local_ctrller = ModelPredictiveController(
            self_name,
            start_index=start_index)

    if (local_ctrller == None):
        raise Exception("Invalid Controller Argument")

    auton_system = AutonSystem(
        trajectory,
        local_ctrller,
        self_name,
        other_name,
        left_curb,
        profile
    )

    while not rospy.is_shutdown():
        rospy.spin()
