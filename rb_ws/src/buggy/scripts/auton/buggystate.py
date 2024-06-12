import numpy as np
import utm
from nav_msgs.msg import Odometry as ROSOdom
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Float64, Bool


import rospy


class BuggyState:
    """
    Basically a translator from ROSOdom to ensure everything is in the correct units.

    Other files should ONLY interface with the nav/odom messages through this file!

    (functionally: this should replace pose and world)

    """
    # TODO: point of discussion: do we want to create a custom message for "utm_pose" since we are constantly passing around easting, northing, yaw? or do we just want to create a really simple wrapper for utm_pose with some translators to more publishable ros poses? (kind of in favor of this...)
    #
    def __init__(self, name = "sc"):
        self.use_gps = False
        self.rosodom : ROSOdom = None
        self.gpspose : PoseStamped = None

        # to report if the filter position has separated (so we need to use the antenna position)
        rospy.Subscriber(name + "/debug/filter_gps_seperation_status", Bool, self.update_use_gps)

        rospy.Subscriber(name + "/nav/odom", ROSOdom, self.update_odom)

        # TODO: this node is reading in a posestamped, do we want to translate HERE or translate in telematics?
        self.gnss_subscriber = rospy.Subscriber(name + "/gnss1/fix_Pose", PoseStamped, self.update_gnss)

    def update_use_gps(self, msg):
        self.use_gps = msg

    def update_odom(self, msg):
        self.rosodom = msg

    def update_gnss(self, msg):
        self.gpspose = msg

    def get_pose(self):
        if self.rosodom == None:
            return None

        if self.use_gps:
            return self.gpspose.pose

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
