#!/usr/bin/env python3

from abc import ABC, abstractmethod

from threading import Lock

import numpy as np

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from pose import Pose


class RTKErrPublisher():

    """
    Publish distance between front antenna and filtered output
    """

    def __init__(self, self_name) -> None:
        self.odom_subscriber = rospy.Subscriber(self_name + "/nav/odom", Odometry, self.update_odom)
        self.gnss_subscriber = rospy.Subscriber("/gnss2/fix_Pose", PoseStamped, self.update_gnss1)

        self.distance_publisher = rospy.Publisher(
            self_name + "/gnss_odom_distance", Float64, queue_size=1
        )


        self.rate = 100
        self.odom_msg = None
        self.gnss1_msg = None
        self.lock = Lock()

    def update_odom(self, msg):
        with self.lock:
            self.odom_msg = msg

    def update_gnss1(self, msg):
        with self.lock:
            self.gnss1_msg = msg

    def loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            with self.lock:
                odom_pose, _ = Pose.rospose_to_pose(self.odom_msg.pose.pose)
                gnss1_pose = Pose.rospose_to_pose(self.gnss1_msg.pose)

            distance = (odom_pose.x - gnss1_pose.x) ** 2 + (odom_pose.y - gnss1_pose.y) ** 2
            distance = np.sqrt(distance)
            self.distance_publisher.publish(Float64(distance))
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("publish_rtk_err")
    d = RTKErrPublisher()
    d.loop()
