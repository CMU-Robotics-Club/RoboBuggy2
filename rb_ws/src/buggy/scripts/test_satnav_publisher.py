import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import NavSatFix


def messagePublisher():
    message_publisher = rospy.Publisher("GPS", NavSatFix, queue_size=10)
    rospy.init_node("test_satnav_publisher", anonymous=True)
    rate = rospy.Rate(2) # publish rate (Hz)
    s = 0
    while not rospy.is_shutdown():
        m = NavSatFix()
        m.header = Header()
        m.status = NavSatStatus()
        m.latitude = 1.0
        m.longitude = 2.0
        m.altitude = 3.0
        rospy.loginfo(m)
        message_publisher.publish(m)
        s += 1
        rate.sleep()

if __name__ == "__main__":
    try:
        messagePublisher()
    #capture the Interrupt signals
    except rospy.ROSInterruptException:
        pass