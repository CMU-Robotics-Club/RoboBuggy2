#! /usr/bin/env python3

import sys
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Int8MultiArray, Int8
from microstrain_inertial_msgs.msg import FilterStatus, ImuOverrangeStatus
from geometry_msgs.msg import PoseStamped

class SanityCheck:
    def __init__(self,
            self_name,
            ) -> None:

        self.covariance = 0

        self.filter_location = None
        self.gps_location = None

        self.imu_overrange_status = None

        self.status_flag_val : int = 0
        self.flags = []
        # string list where indices match the meaning of relevant bits
        self.error_messages : list = ["filter unstable", "filter converging", "roll/pitch warning", "heading warning", "position warning", "velocity warning", "IMU bias warning", "gnss clock warning", "antenna lever arm warning", "mounting transform warning", "solution error", "solution error", "solution error", "solution error", "solution error"]

        # filter seperation = 2, other error = 1, nothing wrong = 0
        self.warning = 0

        self.warning_buffer_time = 1000 # warning flags active >1 second before human driver alerted

        self.warning_durations = [0] * 18 # keeps track of how long covariance, overrange warning, any of the filter status flags are active

        rospy.Subscriber(self_name + "/imu/overrange_status", ImuOverrangeStatus, self.update_overrange_status)
        rospy.Subscriber(self_name + "/nav/status.status_flags", FilterStatus, self.update_status_flags)
        rospy.Subscriber(self_name + "/gnss1/fix_Pose/", PoseStamped, self.update_gps_location)
        rospy.Subscriber(self_name + "/nav/odom", Odometry, self.update_filter_location)


        # these publishers are all bools as quick sanity checks (can display as indicators on foxglove for colors)
        self.overrange_status_publisher = rospy.Publisher(self_name + "/debug/imu_overrange_status", Bool, queue_size=1)

        self.filter_gps_status_publisher = rospy.Publisher(self_name + "/debug/filter_gps_seperation_status", Bool, queue_size=1)

        self.covariance_status_publisher = rospy.Publisher(self_name + "/debug/covariance_status", Bool, queue_size=1)

        self.error_message_publisher = rospy.Publisher(
            self_name + "/nav/status/error_messages", String, queue_size=1
        )

        self.status_flags_publisher = rospy.Publisher(
            self_name + "/nav/status/tripped_status_flags", Int8MultiArray, queue_size=1
        )

        self.overall_warning_publisher = rospy.Publisher(
            self_name + "/debug/sanity_warning", Int8, queue_size=1
        )

    def update_overrange_status(self, msg : ImuOverrangeStatus):
        self.imu_overrange_status = msg.status

    def update_status_flags(self, msg : FilterStatus):
        self.status_flag_val = msg.gq7_status_flags

    def update_gps_location(self, msg : PoseStamped):
        self.gps_location = msg.pose

    def update_filter_location(self, msg):
        self.filter_location = msg.pose

    def update_warning_flags (self, idx):
        self.warning_durations[idx] += 10
        if (self.warning_durations[idx] > self.warning_buffer_time):
            self.warning = 1

    def calc_covariance(self):
        if (self.filter_location == None):
            self.covariance_status_publisher.publish(False)
        else:
            good_covariance = self.filter_location.covariance[0] ** 2 + self.filter_location.covariance[7] ** 2 <= 1**2
            if (not good_covariance):
                self.update_warning_flags(16)
            else:
                self.warning_durations[16] = 0
            self.covariance_status_publisher.publish(good_covariance)


    def calc_locations(self):
        # currently comparing cross-track error, checking less than 0.5 m
        if (self.filter_location == None or self.gps_location == None):
            self.filter_gps_status_publisher.publish(False)
        else:
            good_seperation = abs(self.filter_location.pose.position.y - self.gps_location.position.y) < 0.5
            if (not good_seperation):
                self.warning = 2

            self.filter_gps_status_publisher.publish(good_seperation)

    def is_overrange (self):
        s = self.imu_overrange_status
        if (s == None):
            self.overrange_status_publisher.publish(False)
        else:
            accel_status = s.status_accel_x or s.status_accel_y or s.status_accel_z
            gyro_status = s.status_gyro_x or s.status_gyro_y or s.status_gyro_z
            mag_status = s.status_mag_x or s.status_mag_y or s.status_mag_z
            is_overrange = accel_status or gyro_status or mag_status or s.status_press
            if (is_overrange):
                self.update_warning_flags(17)
            else:
                self.warning_durations[17] = 0
            # publishes true if ALL flags are FALSE (if the imu looks good)
            self.imu_overrange_status.publish(not is_overrange)

    def filter_status_warning (self):
        b = bin(self.status_flag_val)
        self.flags = Int8MultiArray()
        self.flags.data = []
        error_message = ""

        # reading filter condition bits
        if (b[0] == 1 and b[1] == 1): # filter unstable
            self.update_warning_flags(0)
            self.flags.append(0)
            self.flags.append(1)
            error_message += self.error_messages[0] + " "

        if (b[0] == 1 and b[1] == 0): # filter converging
            self.update_warning_flags(1)
            self.flags.append(0)
            error_message += self.error_messages[1] + " "

        for i in range (len(b)):
            if (i >= 2): # if not reading filter condiiton bits (already considered)
                if (b[i] == '1'):
                    self.update_warning_flags(i)
                    self.flags.append(i)
                    error_message += self.error_messages[i] + " "
                else:
                    self.warning_durations[i] = 0
        # publish bit values
        self.status_flags_publisher.publish(self.flags)

        # publish string translations
        self.error_message_publisher.publish(error_message)

    def sanity_check(self):
        self.warning = False
        self.calc_covariance()
        self.calc_locations()
        self.is_overrange()
        self.filter_status_warning()
        self.overall_warning_publisher.publish(self.warning)

if __name__ == "__main__":
    rospy.init_node("rolling_sanity_check")
    check = SanityCheck(sys.argv[1])
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        check.sanity_check()
        rate.sleep()