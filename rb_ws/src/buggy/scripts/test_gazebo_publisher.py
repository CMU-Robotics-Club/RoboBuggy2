import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
def messagePublisher():
    message_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.init_node("buggyGazeboTestPublisher", anonymous=True)
    rate = rospy.Rate(2) # publish rate (Hz)
    while not rospy.is_shutdown():
        # rospy.loginfo("Published: " + message)
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = 1
        message_publisher.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        messagePublisher()
    #capture the Interrupt signals
    except rospy.ROSInterruptException:
        pass