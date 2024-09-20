#! /usr/bin/env python3
# Runs the conversion script for all telematics data
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int8

from microstrain_inertial_msgs.msg import MipGnssFixInfo

class Telematics:
    """
    Converts subscribers and publishers that need to be reformated, so that they are readible.
    """

    def __init__(self):
        """Generate all the subscribers and publishers that need to be reformatted.
        """

        self.odom_subscriber = rospy.Subscriber("/NAND/nav/odom", Odometry, self.convert_odometry_to_navsatfix)
        self.odom_publisher = rospy.Publisher("/NAND/nav/odom_NavSatFix", NavSatFix, queue_size=10)

        self.gnss1_pose_publisher = rospy.Publisher("/gnss1/fix_Pose", PoseStamped, queue_size=10)
        self.gnss1_covariance_publisher = rospy.Publisher("/gnss1/fix_FloatArray_Covariance", Float64MultiArray, queue_size=10)
        self.gnss1_subscriber = rospy.Subscriber("/gnss1/llh_position", NavSatFix, self.convert_navsatfix_to_pose_covariance, (self.gnss1_pose_publisher, self.gnss1_covariance_publisher))

        self.gnss2_pose_publisher = rospy.Publisher("/gnss2/fix_Pose", PoseStamped, queue_size=10)
        self.gnss2_covariance_publisher = rospy.Publisher("/gnss2/fix_FloatArray_Covariance", Float64MultiArray, queue_size=10)
        self.gnss2_subscriber = rospy.Subscriber("/gnss2/llh_position", NavSatFix, self.convert_navsatfix_to_pose_covariance, (self.gnss2_pose_publisher, self.gnss2_covariance_publisher))

        self.gnss1_fixinfo_publisher = rospy.Publisher("/gnss1/fix_info_republished", String, queue_size=10)
        self.gnss1_fixinfo_int_publisher = rospy.Publisher("/gnss1/fix_info_republished_int", Int8, queue_size=10)
        self.gnss1_fixinfo_subscriber = rospy.Subscriber("/mip/gnss1/fix_info", MipGnssFixInfo, self.republish_fixinfo, (self.gnss1_fixinfo_publisher, self.gnss1_fixinfo_int_publisher))

        self.gnss2_fixinfo_publisher = rospy.Publisher("/gnss2/fix_info_republished", String, queue_size=10)
        self.gnss2_fixinfo_int_publisher = rospy.Publisher("/gnss2/fix_info_republished_int", Int8, queue_size=10)
        self.gnss2_fixinfo_subscriber = rospy.Subscriber("/mip/gnss2/fix_info", MipGnssFixInfo, self.republish_fixinfo, (self.gnss2_fixinfo_publisher, self.gnss2_fixinfo_int_publisher))

    def convert_odometry_to_navsatfix(self, msg):
        """Convert Odometry-type to NavSatFix-type for plotting on Foxglove
        Args:
            msg (Odometry): odometry as per INS
        """
        lat = msg.pose.pose.position.y
        long = msg.pose.pose.position.x
        down = msg.pose.pose.position.z
        new_msg = NavSatFix()
        new_msg.header = msg.header
        new_msg.latitude = lat
        new_msg.longitude = long
        new_msg.altitude = down
        self.odom_publisher.publish(new_msg)

    def convert_navsatfix_to_pose_covariance(self, msg, publishers):
        """Convert NavSatFix-type and related covariance matrix to Pose-type and array
        respectively for easy visualization in Foxglove.

        Args:
            msg (NavSatFix): original msg
            publishers (tuple): tuple of publishes
        """
        pose = PoseStamped()
        pose.pose.position.y = msg.latitude
        pose.pose.position.x = msg.longitude
        pose.pose.position.z = msg.altitude
        pose.header = msg.header
        publishers[0].publish(pose)

        covariance = Float64MultiArray()
        covariance.data = [
            round(msg.position_covariance[0], 4),
            round(msg.position_covariance[4], 4),
            round(msg.position_covariance[8], 4)]
        publishers[1].publish(covariance)

    def republish_fixinfo(self, msg, publishers):
        """
        convert gnss/fixinfo to a string for visualization in foxglove
        """
        fix_type = msg.fix_type
        fix_string = "fix type: "

        if (fix_type == 0):
            fix_string += "FIX_3D"
        elif (fix_type == 1):
            fix_string += "FIX_2D"
        elif (fix_type == 2):
            fix_string += "FIX_TIME_ONLY"
        elif (fix_type == 3):
            fix_string += "FIX_NONE"
        elif (fix_type == 4):
            fix_string += "FIX_INVALID"
        elif (fix_type == 5):
            fix_string += "FIX_RTK_FLOAT"
        else:
            fix_string += "FIX_RTK_FIXED"

        fix_string += "\nsbas_used: "  + str(msg.sbas_used)
        fix_string += "\ndngss_used: " + str(msg.dngss_used)
        publishers[0].publish(fix_string)
        publishers[1].publish(fix_type)

if __name__ == "__main__":
    rospy.init_node("telematics")
    telem = Telematics()
    rospy.spin()
