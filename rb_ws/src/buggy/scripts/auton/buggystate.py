import utm
from nav_msgs.msg import Odometry as ROSOdom
from sensor_msgs.msg import NavSatFix

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Bool

from microstrain_inertial_msgs.msg import GNSSFixInfo



import rospy


class BuggyState:
    """
    Basically a translator from ROSOdom to ensure everything is in the correct units.

    Other files should ONLY interface with the nav/odom messages through this file!

    (functionally: this should replace telematics, pose and world)

    """

    def __init__(self, name = "sc"):
        self.filter_odom : ROSOdom = None
        self.gnss_1 : ROSOdom = None
        self.gnss_2 : ROSOdom = None
        self.gps_fix : int = 0


        # to report if the filter position has separated (so we need to break)
        rospy.Subscriber(name + "/debug/filter_gps_seperation_status", Bool, self.update_use_gps)

        rospy.Subscriber(name + "/nav/odom", ROSOdom, self.update_odom)
        rospy.Subscriber("/gnss1/odom", ROSOdom, self.update_gnss1)
        rospy.Subscriber("/gnss2/odom", ROSOdom, self.update_gnss2)
        rospy.Subscriber("/gnss1/fix_info", GNSSFixInfo, self.update_gnss1_fix)


    def update_use_gps(self, msg):
        self.use_gps = msg
    def update_odom(self, msg):
        self.filter_odom = msg
    def update_gnss1(self, msg):
        self.gnss_1 = msg
    def update_gnss2(self, msg):
        self.gnss_2 = msg
    def update_gnss1_fix(self, msg):
        self.gps_fix = msg.fix_type

    def odom_to_navsatfix(self, odom):
        """Convert Odometry-type to NavSatFix-type for plotting on Foxglove
        Args:
            odom (Odometry): odometry object to convert
        """
        lat = odom.pose.pose.position.y
        long = odom.pose.pose.position.x
        down = odom.pose.pose.position.z
        new_odom = NavSatFix()
        new_odom.header = odom.header
        new_odom.latitude = lat
        new_odom.longitude = long
        new_odom.altitude = down
        return new_odom

    def get_gps_fix(self):
        fix_string = "fix type: "
        if (self.gps_fix == 0):
            fix_string += "FIX_3D"
        elif (self.gps_fix == 1):
            fix_string += "FIX_2D"
        elif (self.gps_fix == 2):
            fix_string += "FIX_TIME_ONLY"
        elif (self.gps_fix == 3):
            fix_string += "FIX_NONE"
        elif (self.gps_fix == 4):
            fix_string += "FIX_INVALID"
        elif (self.gps_fix == 5):
            fix_string += "FIX_RTK_FLOAT"
        else:
            fix_string += "FIX_RTK_FIXED"
        return fix_string

    def get_pose(self):
        if self.filter_odom == None:
            return None

        rospose = self.filter_odom.pose.pose
        (_, _, yaw) = euler_from_quaternion(
            [
                rospose.orientation.x,
                rospose.orientation.y,
                rospose.orientation.z,
                rospose.orientation.w,
            ]
        )

        (easting, northing, _, _) = utm.from_latlon(rospose.position.y, rospose.position.x)

        return (easting, northing, yaw)

    def get_pos_covariance(self):
        if self.filter_odom == None:
            return None
        return self.filter_odom.pose.covariance

    def get_velocity(self):
        if self.filter_odom == None:
            return None
        return self.filter_odom.twist.twist.linear

