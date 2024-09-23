#!/usr/bin/env python3

from threading import Lock
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from pose import Pose
from world import World


#BROKEN BROKEN BROKEN BROKEN
class RTKErrPublisher():

    """
    Publish distance between front antenna and filtered output. Literally just a file to publish data via ROS for other classes to use.
    """
    # TODO: pretty sure this isn't used anymore (we publish odom data for each buggy independently in autonsystem anyway, this node is only launched in debug) but wanted a second opinion since basically the entire stack is based on this data getting published correctly

    def __init__(self) -> None:
        self.odom_subscriber = rospy.Subscriber("/nav/odom", Odometry, self.update_odom)
        self.gnss_subscriber = rospy.Subscriber("/gnss2/fix_Pose", PoseStamped, self.update_gnss2)

        self.distance_publisher = rospy.Publisher(
            "/gnss_odom_distance", Float64, queue_size=1
        )

        self.rate = 100
        self.odom_msg = None
        self.gnss2_msg = None
        self.lock = Lock()


    def update_odom(self, msg):
        with self.lock:
            self.odom_msg = msg

    def update_gnss2(self, msg):
        with self.lock:
            self.gnss2_msg = msg

        if not (self.odom_msg is None) and not (self.gnss2_msg is None):
            odom_pose = World.gps_to_world_pose(Pose.rospose_to_pose(self.odom_msg.pose.pose))
            gnss2_pose = World.gps_to_world_pose(Pose.rospose_to_pose(self.gnss2_msg.pose))

            distance = (odom_pose.x - gnss2_pose.x) ** 2 + (odom_pose.y - gnss2_pose.y) ** 2
            distance = np.sqrt(distance)

            self.distance_publisher.publish(Float64(distance))

if __name__ == "__main__":
    rospy.init_node("publish_rtk_err")
    d = RTKErrPublisher()
    rospy.spin()