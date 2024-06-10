import numpy as np
import utm
from nav_msgs.msg import Odometry as ROSOdom
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

import rospy


class BuggyState:
    """
    Basically a translator from ROSOdom to ensure everything is in the correct units.

    Other files should ONLY interface with the nav/odom messages through this file!

    (functionally: this should replace pose and world)

    """
    def __init__(self):
        self.rosodom : ROSOdom = None
        rospy.Subscriber("/nav/odom", ROSOdom, self.update_odom)

    def update_odom(self, msg):
        self.rosodom = msg

    def get_pose(self):
        if self.rosodom == None:
            return None

        rospose = self.rosodom.pose.pose
        yaw = (_, _, yaw) = euler_from_quaternion(
            [
                rospose.orientation.x,
                rospose.orientation.y,
                rospose.orientation.z,
                rospose.orientation.w,
            ]
        )

        easting, northing = utm.from_latlon(rospose.position.x, rospose.position.y)

        return (easting, northing, yaw)


    def get_pos_covariance(self):
        if self.rosodom == None:
            return None
        return self.rosodom.pose.covariance

    # TODO: finish this!
    def get_velocity(self):
        if self.rosodom == None:
            return None
        return None

    # TODO: finish this!
    def create_odom(self):
        return None
