#!/usr/bin/env python3

import argparse
from threading import Lock

import threading
import rospy

# ROS Message Imports
from std_msgs.msg import Float32, Float64, Bool, Int8
from nav_msgs.msg import Odometry

import numpy as np

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
    brake_controller: BrakeController = None
    lock = None
    steer_publisher = None
    ticks = 0

    def __init__(self,
            global_trajectory,
            local_controller,
            brake_controller,
            self_name,
            other_name,
            profile) -> None:


        self.global_trajectory = global_trajectory

        # local trajectory is initialized as global trajectory. If there is no other buggy,
        # the local trajectory is never updated.

        self.has_other_buggy = not other_name is None
        self.cur_traj = global_trajectory
        self.local_controller = local_controller
        self.brake_controller = brake_controller

        left_curb = Trajectory(json_filepath="/rb_ws/src/buggy/paths/garage_inner_bound.json")
        self.path_planner = PathPlanner(global_trajectory, left_curb)
        self.other_steering = 0
        self.rtk_status = 0

        self.lock = Lock()
        self.ticks = 0
        self.self_odom_msg = None
        self.other_odom_msg = None

        rospy.Subscriber(self_name + "/nav/odom", Odometry, self.update_self_odom)
        if self.has_other_buggy:
            rospy.Subscriber(other_name + "/nav/odom", Odometry, self.update_other_odom)
            self.other_steer_subscriber = rospy.Subscriber(
                other_name + "/buggy/input/steering", Float64, self.update_other_steering_angle
            )
        rospy.Subscriber(self_name + "/gnss1/fix_info_republished_int", Int8, self.update_rtk_status)

        self.init_check_publisher = rospy.Publisher(
            self_name + "/debug/init_safety_check", Bool, queue_size=1
        )
        self.steer_publisher = rospy.Publisher(
            self_name + "/buggy/input/steering", Float64, queue_size=1
        )
        self.brake_publisher = rospy.Publisher(
            self_name + "/buggy/input/brake", Float64, queue_size=1
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


        self.controller_rate = 100
        self.rosrate_controller = rospy.Rate(self.controller_rate)

        self.planner_rate = 10
        self.rosrate_planner = rospy.Rate(self.planner_rate)

        self.profile = profile
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

    def update_rtk_status(self, msg):
        with self.lock:
            self.rtk_status = msg.data

    def init_check(self):
        # checks that messages are being receieved
        # (from both buggies if relevant),
        # RTK status is fixed
        # covariance is less than 1 meter
        if (self.self_odom_msg == None) or (self.has_other_buggy and self.other_odom_msg == None) or (self.self_odom_msg.pose.covariance[0] ** 2 + self.self_odom_msg.pose.covariance[7] ** 2 > 1**2):
            return False

        # waits until covariance is acceptable to check heading

        with self.lock:
            self_pose, _ = self.get_world_pose_and_speed(self.self_odom_msg)
            current_heading = self_pose.theta
            closest_heading = self.cur_traj.get_heading_by_index(trajectory.get_closest_index_on_path(self_pose.x, self_pose.y))

        # TENTATIVE:
        # headings are originally between -pi and pi
        # if they are negative, convert them to be between 0 and pi
        if current_heading < 0:
            current_heading = 2*np.pi + current_heading

        if closest_heading < 0:
            closest_heading = 2*np.pi + closest_heading

        if (abs(current_heading - closest_heading) >= np.pi/2):
            print("WARNING: INCORRECT HEADING! restart stack")
            return False

        return True

    def tick_caller(self):

        while ((not rospy.is_shutdown()) and not self.init_check()):
            self.init_check_publisher.publish(False)
            rospy.sleep(0.001)
        print("done checking initialization status")
        self.init_check_publisher.publish(True)


        # initialize global trajectory index

        with self.lock:
            _, _ = self.get_world_pose_and_speed(self.self_odom_msg)

        p2 = threading.Thread(target=self.planner_thread)
        p1 = threading.Thread(target=self.local_controller_thread)

        # starting processes
        # See LOOKAHEAD_TIME in path_planner.py for the horizon of the
        # planner. Make sure it is significantly (at least 2x) longer
        # than 1 period of the planner when you change the planner frequency.
        p2.start() #Planner runs every 10 hz
        p1.start() #Main Cycles runs at 100hz

        p2.join()
        p1.join()

    def get_world_pose_and_speed(self, msg):
        current_rospose = msg.pose.pose
        # Check if the pose covariance is a sane value. Publish a warning if insane
        current_speed = np.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )

        # Get data from message
        pose_gps = Pose.rospose_to_pose(current_rospose)
        return World.gps_to_world_pose(pose_gps), current_speed

    def local_controller_thread(self):
        while (not rospy.is_shutdown()):
            self.local_controller_tick()
            self.rosrate_controller.sleep()

    def local_controller_tick(self):
        with self.lock:
            self_pose, self_speed = self.get_world_pose_and_speed(self.self_odom_msg)

        # Compute control output
        steering_angle = self.local_controller.compute_control(
            self_pose, self.cur_traj, self_speed)
        steering_angle_deg = np.rad2deg(steering_angle)
        self.steer_publisher.publish(Float64(steering_angle_deg))


    def planner_thread(self):
        while (not rospy.is_shutdown()):
            if not self.other_odom_msg is None:
                with self.lock:
                    self_pose, _ = self.get_world_pose_and_speed(self.self_odom_msg)
                    other_pose, _ = self.get_world_pose_and_speed(self.other_odom_msg)
                    distance = (self_pose.x - other_pose.x) ** 2 + (self_pose.y - other_pose.y) ** 2
                    distance = np.sqrt(distance)
                    self.distance_publisher.publish(Float64(distance))

                self.planner_tick()
            self.rosrate_planner.sleep()


    def planner_tick(self):
        with self.lock:
            self_pose, _ = self.get_world_pose_and_speed(self.self_odom_msg)
            other_pose, other_speed = self.get_world_pose_and_speed(self.other_odom_msg)

        # update local trajectory via path planner
        self.cur_traj, cur_idx = self.path_planner.compute_traj(
                                            self_pose,
                                            other_pose)
        self.local_controller.current_traj_index = cur_idx

if __name__ == "__main__":
    rospy.init_node("auton_system")
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
        "--other_name",
        type=str,
        help="name of other buggy, if left unspecified, the autonsystem assumes it is the only buggy on the course",
        required=False)

    parser.add_argument(
        "--profile",
        action='store_true',
        help="turn on profiling for the path planner")

    args, _ = parser.parse_known_args()
    ctrl = args.controller
    start_dist = args.dist
    traj = args.traj
    self_name = args.self_name
    other_name = args.other_name
    profile = args.profile

    print("\n\nStarting Controller: " + str(ctrl) + "\n\n")
    print("\n\nUsing path: /rb_ws/src/buggy/paths/" + str(traj) + "\n\n")
    print("\n\nStarting at distance: " + str(start_dist) + "\n\n")

    trajectory = Trajectory(json_filepath="/rb_ws/src/buggy/paths/" + traj)

    # calculate starting index
    start_index = trajectory.get_index_from_distance(start_dist)

    # Add Controllers Here
    local_ctrller = None
    if (ctrl == "stanley"):
        local_ctrller = StanleyController(
            self_name,
            start_index=start_index)

    elif (ctrl == "pure_pursuit"):
        local_ctrller = PurePursuitController(
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
        BrakeController(),
        self_name,
        other_name,
        profile
    )

    while not rospy.is_shutdown():
        rospy.spin()
