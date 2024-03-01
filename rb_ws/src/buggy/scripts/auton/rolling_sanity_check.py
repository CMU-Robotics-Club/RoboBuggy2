import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64, Bool, Int8

import rospy

from brake_controller import BrakeController
from auton.controller import Controller
from trajectory import Trajectory

class SanityCheck:
    #TODO: copied from autonsystem, delete uneeded args
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
        self.has_other_buggy = not other_name is None

        self.cur_traj = global_trajectory
        self.local_controller = local_controller
        self.brake_controller = brake_controller
        self.rtk_status = 0

        self.covariance = 0
        self.location = None
        self.filter_location = None


        #TODO: do we need lock
        self.lock = Lock()
        self.self_odom_msg = None
        self.other_odom_msg = None

        rospy.Subscriber(self_name + "/nav/odom", Odometry, self.update_self_odom)

        if self.has_other_buggy:
            rospy.Subscriber(other_name + "/nav/odom", Odometry, self.update_other_odom)

        rospy.Subscriber(self_name + "/gnss1/fix_info_republished_int", Int8, self.update_rtk1_status)

        rospy.Subscriber(self_name + "/gnss2/fix_info_republished_int", Int8, self.update_rtk2_status)


    def update_self_odom(self, msg):
        with self.lock:
            self.self_odom_msg = msg

    def update_other_odom(self, msg):
        with self.lock:
            self.other_odom_msg = msg


    def calc_covariance(self):
        self.covariance = self.self_odom_msg.pose.covariance[0] ** 2 + self.self_odom_msg.pose.covariance[7] ** 2

    def sanity_check(self):
        if (abs(self.filter_location - self.location) > 0):
            print("filter and gps seperate")

        if (self.covariance > 1**2):
            print("covariance bad")

        # TO CHECK:
        #     filter-GPS separation
        #     Covariance

        #     IMU overrange warning
        #     Converts status flags to strings and publish
        #     Consult https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/Home.htm
        # also add node to launchfiles
