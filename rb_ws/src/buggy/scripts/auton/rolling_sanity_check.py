#! /usr/bin/env python3

import sys
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Int8MultiArray

import rospy

from microstrain_inertial_msgs.msg import FilterStatus
from microstrain_inertial_msgs.msg import ImuOverrangeStatus
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
        self.error_messages : list = ["filter stable/recovering", "filter converging", "roll/pitch warning", "heading warning", "position warning", "velocity warning", "IMU bias warning", "gnss clock warning", "antenna lever arm warning", "mounting transform warning", "solution error", "solution error", "solution error", "solution error", "solution error"]


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



    def update_overrange_status(self, msg : ImuOverrangeStatus):
        self.imu_overrange_status = msg.status

    def update_status_flags(self, msg : FilterStatus):
        self.status_flag_val = msg.gq7_status_flags

    def update_gps_location(self, msg : PoseStamped):
        self.gps_location = msg.pose

    def update_filter_location(self, msg):
        self.filter_location = msg.pose

    def calc_covariance(self):
        if (self.filter_location == None):
            self.covariance_status_publisher.publish(False)
        else:
            self.covariance_status_publisher.publish(self.filter_location.covariance[0] ** 2 + self.filter_location.covariance[7] ** 2 <= 1**2)

    def calc_locations(self):
        #TODO: what data should we store/ use to compare filter location
        # currently comparing cross-track error, checking less than 0.5 m
        if (self.filter_location == None or self.gps_location == None):
            self.filter_gps_status_publisher.publish(False)
        else:
            self.filter_gps_status_publisher.publish(abs(self.filter_location.pose.position.y - self.gps_location.position.y) < 0.5)

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
