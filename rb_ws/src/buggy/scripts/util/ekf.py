#!/usr/bin/env python3

import sys

# Allows import of world and pose from auton directory
# sys.path.append("/rb_ws/src/buggy/scripts/auton")

# from buggystate import BuggyState
from threading import Lock
import rospy

# Ros Message Imports
from geometry_msgs.msg import PoseWithCovarianceStamped as ROSOdom
from sensor_msgs.msg import NavSatFix


class ekfTranslator:
    def __init__(self):
        """
        self_name: Buggy namespace
        other_name: Only requried by SC for passing buggy namespace
        teensy_name: required for communication, different for SC and NAND

        Initializes the subscribers, rates, and ros topics (including debug topics)
        """
        self.odomMsg = None

        rospy.Subscriber(
            "/robot_pose_ekf/odom_combined", ROSOdom, self.set_odom
        )

        self.lock = Lock()

        self.publisher = rospy.Publisher("/robot_pose_ekf/navsatfix", NavSatFix, queue_size=1)

    def set_odom(self, msg):
        with self.lock:
            lat = msg.pose.pose.position.y
            long = msg.pose.pose.position.x
            down = msg.pose.pose.position.z
            new_odom = NavSatFix()
            new_odom.header = msg.header
            new_odom.latitude = lat
            new_odom.longitude = long
            new_odom.altitude = down
            self.publisher.publish(new_odom)
            print("ekf Translated")


if __name__ == "__main__":
    rospy.init_node("ekf_translator")
    translate = ekfTranslator()
    print("Initialized EKF Translator")
    x = 1
    while True:
        x+=1