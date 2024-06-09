import numpy as np
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
        rospy.Subscriber("/nav/odom", ROSOdom, self.get_odom)

    def get_odom(self, msg):
        self.rosodom = msg

    """
    FUNCTIONALITY TO ADD:
        get pose (in utm)
        get velocity (in utm)
        get covariance (in whatever units)
        create a new rosodom message using some (maybe not all) information (by default set missing info to garbage vals)

        how does this interact with gps coords and all
        can we just make this combine with that
    """