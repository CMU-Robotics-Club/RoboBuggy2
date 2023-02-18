#! /usr/bin/env python3
# Runs the conversion script for all telematics data
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray


class Telematics:
    def __init__(self):
        """Generate all the subscribers and publishers that need to be reformatted.
        """
        self.odom_subscriber = rospy.Subscriber("/nav/odom", Odometry, self.convert_odometry_to_navsatfix)
        self.odom_publisher = rospy.Publisher("/nav/odom_NavSatFix", NavSatFix, queue_size=10)

        self.gnss1_pose_publisher = rospy.Publisher("/gnss1/fix_Pose", PoseStamped, queue_size=10)
        self.gnss1_covariance_publisher = rospy.Publisher("/gnss1/fix_FloatArray_Covariance", Float64MultiArray, queue_size=10)
        self.gnss1_subscriber = rospy.Subscriber("/gnss1/fix", NavSatFix, self.convert_navsatfix_to_pose_covariance, (self.gnss1_pose_publisher, self.gnss1_covariance_publisher))

        self.gnss2_pose_publisher = rospy.Publisher("/gnss2/fix_Pose", PoseStamped, queue_size=10)
        self.gnss2_covariance_publisher = rospy.Publisher("/gnss2/fix_FloatArray_Covariance", Float64MultiArray, queue_size=10)
        self.gnss2_subscriber = rospy.Subscriber("/gnss2/fix", NavSatFix, self.convert_navsatfix_to_pose_covariance, (self.gnss2_pose_publisher, self.gnss2_covariance_publisher))


    
    def convert_odometry_to_navsatfix(self, msg):
        """Convert Odometry-type to NavSatFix-type for plotting on Foxglove

        Args:
            msg (Odometry): odometry as per INS
        """
        lat = msg.pose.pose.position.x
        long = msg.pose.pose.position.y
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
        pose.pose.position.x = msg.latitude
        pose.pose.position.y = msg.longitude
        pose.pose.position.z = msg.altitude
        pose.header = msg.header
        publishers[0].publish(pose)

        covariance = Float64MultiArray()
        covariance.data = [msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8]]
        publishers[1].publish(covariance)

if __name__ == "__main__":
  rospy.init_node("telematics")
  telem = Telematics()
  rospy.spin()
  