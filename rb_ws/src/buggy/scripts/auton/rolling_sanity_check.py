#! /usr/bin/env python3

import numpy as np
import sys
from nav_msgs.msg import Odometry
from pose import Pose
from world import World
from std_msgs.msg import Float32, Float64, Bool, Int8, String, Int8MultiArray

import rospy

from brake_controller import BrakeController
from controller import Controller
from trajectory import Trajectory
from microstrain_inertial_msgs.msg import FilterStatus
from microstrain_inertial_msgs.msg import ImuOverrangeStatus
from sensor_msgs.msg import Imu


class SanityCheck:
    def __init__(self,
            self_name,
            ) -> None:

        self.covariance = 0

        self.self_odom_msg = None
        self.location = None
        self.filter_location = None

        self.imu_overrange_status = None
        self.status_flag_val : int = 0
        self.flags = []

        # string list where indices match the meaning of relevant bits
        self.error_messages : list = ["filter stable/recovering", "filter converging", "roll/pitch warning", "heading warning", "position warning", "velocity warning", "IMU bias warning", "gnss clock warning", "antenna lever arm warning", "mounting transform warning", "solution error", "solution error", "solution error", "solution error", "solution error"]


        rospy.Subscriber(self_name + "/nav/odom", Odometry, self.update_self_odom)
        rospy.Subscriber(self_name + "/imu/overrange_status", ImuOverrangeStatus, self.update_overrange_status)
        rospy.Subscriber(self_name + "/nav/filtered_imu/data", Imu, self.update_filter_location)

        rospy.Subscriber(self_name + "/nav/status.status_flags", FilterStatus, self.update_status_flags)

        # these publishers are all bools as quick sanity checks
        self.overrange_status_publisher = rospy.Publisher(self_name + "/debug/imu_overrange_status", Bool, queue_size=1)

        self.filter_gps_status_publisher = rospy.Publisher(self_name + "/debug/filter_gps_seperation_status", Bool, queue_size=1)

        self.covariance_status_publisher = rospy.Publisher(self_name + "/debug/covariance_status", Bool, queue_size=1)

        self.error_message_publisher = rospy.Publisher(
            self_name + "/nav/status/error_messages", String, queue_size=1
        )

        self.status_flags_publisher = rospy.Publisher(
            self_name + "/nav/status/tripped_status_flags", Int8MultiArray, queue_size=1
        )

    def update_self_odom(self, msg):
        self.self_odom_msg = msg

    def update_other_odom(self, msg):
        self.other_odom_msg = msg

    def update_overrange_status(self, msg : ImuOverrangeStatus):
        self.imu_overrange_status = msg.status

    def update_status_flags(self, msg : FilterStatus):
        self.status_flag_val = msg.gq7_status_flags

    def update_filter_location(self, msg : Imu):
        #TODO: what data should we store/ use to compare filter location
        if (self.filter_location != None):
            self.filter_location = msg.orientation.x


    def calc_covariance(self):
        if (self.self_odom_msg == None):
            self.covariance_status_publisher.publish(False)
        else:
            self.covariance_status_publisher.publish(self.self_odom_msg.pose.covariance[0] ** 2 + self.self_odom_msg.pose.covariance[7] ** 2 <= 1**2)

    def calc_locations(self):
        #TODO: actually publish relevant data
        if (self.self_odom_msg == None or self.location == None):
            self.filter_gps_status_publisher.publish(False)
        else:
            current_rospose = self.self_odom_msg.pose.pose
            pose_gps = Pose.rospose_to_pose(current_rospose)
            self.location = World.gps_to_world_pose(pose_gps)

            # self.filter_gps_status_publisher.publish(abs(self.filter_location - self.location) < 0.5)

    def is_overrange (self):
        s = self.imu_overrange_status
        if (s == None):
            self.overrange_status_publisher.publish(False)
        else:
            accel_status = s.status_accel_x or s.status_accel_y or s.status_accel_z
            gyro_status = s.status_gyro_x or s.status_gyro_y or s.status_gyro_z
            mag_status = s.status_mag_x or s.status_mag_y or s.status_mag_z

            # publishes true if ALL flags are FALSE (if the imu looks good)
            self.imu_overrange_status.publish(not (accel_status or gyro_status or mag_status or s.status_press))

    def filter_status_warning (self):
        b = bin(self.status_flag_val)
        self.flags = Int8MultiArray()
        self.flags.data = []
        error_message = ""
        for i in range (len(b)):
            if (b[i] == '1'):
                self.flags.append(i)
                error_message += self.error_messages[i] + " "

        # publish bit values
        self.status_flags_publisher.publish(self.flags)

        # publish string translations
        self.error_message_publisher.publish(error_message)

    def sanity_check(self):
        self.calc_covariance()
        self.calc_locations()
        self.is_overrange()
        self.filter_status_warning()

if __name__ == "__main__":
    rospy.init_node("rolling_sanity_check")
    check = SanityCheck(sys.argv[1])
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        check.sanity_check()
        rate.sleep()
