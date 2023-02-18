import rospy
import tf
import time
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



def messagePublisher():
    message_publisher = rospy.Publisher("/gq7/nav/odom", Odometry, queue_size=10)
    rospy.init_node("test_satnav_publisher", anonymous=True)
    rate = rospy.Rate(2) # publish rate (Hz)
    s = 0
    while not rospy.is_shutdown():
        m = Odometry()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "test_satnav_publisher"
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, 90)
        m.pose.pose = Pose(Point(0, 1, 0.), Quaternion(*odom_quat))
        m.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        message_publisher.publish(m)
        s += 1
        rate.sleep()

if __name__ == "__main__":
    try:
        messagePublisher()
    #capture the Interrupt signals
    except rospy.ROSInterruptException:
        pass