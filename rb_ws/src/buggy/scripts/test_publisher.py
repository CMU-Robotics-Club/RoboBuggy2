import rospy
from std_msgs.msg import String
import time
def messagePublisher():
    message_publisher = rospy.Publisher("testMessage", String, queue_size=10)
    message_publisher2 = rospy.Publisher("testMessage2", String, queue_size=10)
    rospy.init_node("buggyTestPublisher", anonymous=True)
    rate = rospy.Rate(2) # publish rate (Hz)
    while not rospy.is_shutdown():
        message = "Robobuggy > Atlas! PublishedTime: " + str(time.time())
        message2 = "2Robobuggy > Atlas! PublishedTime: " + str(time.time())
        rospy.loginfo("Published: " + message)
        message_publisher.publish(message)
        message_publisher.publish(message2)
        rate.sleep()

if __name__ == "__main__":
    try:
        messagePublisher()
    #capture the Interrupt signals
    except rospy.ROSInterruptException:
        pass