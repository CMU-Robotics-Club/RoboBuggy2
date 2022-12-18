#!/usr/bin/python3
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TimeReference
from nav_msgs.msg import Odometry

# records output of Parker gq7 rtk-gps
# sensor in NED relative position form, as well as LLH
# global position form.
# also records other data
# see: https://wiki.ros.org/microstrain_inertial_driver

class SatNavLogger:
    def __init__(self):
        self.file_llh = open("ned_relative_positon_log.txt", "w+")
        self.file_ned = open("llh_global_position_log.txt", "w+")
        self.isLogging = False
        self.timestart = 0

    def start(self):
        self.isLogging = True
        self.listener()
        self.timestart = time.time()

    def writeLine(self, data):
        self.file.write(str(data))
        self.file.write("\n")

    def time_since_start(self):
        return time.time() - self.timestart

    def llhposCb(self, data):
        if self.isLogging:
            self.file_llh.write(str(self.time_since_start()) + str(data) + "\r\n")

    def rposCb(self, data):
        if self.isLogging:
            self.file_ned.write(str(self.time_since_start()) + str(data) + "\r\n")

    def listener(self):
        rospy.init_node("listener", anonymous=True)

        # subscribe to gq7 nodes
        rospy.Subscriber("/gq7/nav/relative_pos_odom", Odometry, self.rposCb)
        rospy.Subscriber("/gq7/nav/odom", Odometry, self.llhposCb)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def stop(self):
        self.isLogging = False
        self.file_llh.close()
        self.file_ned.close()
        

if __name__ == '__main__':
    logger = SatNavLogger()
    logger.start()