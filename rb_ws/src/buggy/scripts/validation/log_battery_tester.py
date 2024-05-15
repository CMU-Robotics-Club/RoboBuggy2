# Script
import rospy
from sensor_msgs.msg import (
    BatteryState,
)  # Callback function to print the subscribed data on the terminal
import numpy as np


def batteryIn_T_publisher():
    """
    Work in progress function to publish the battery tester, should be culled.
    """
    message_publisher = rospy.Publisher("BatteryIn_T", BatteryState, queue_size=10)
    rospy.init_node("batteryLoggerTester", anonymous=False)
    rate = rospy.Rate(2)  # publish rate (Hz)
    while not rospy.is_shutdown():
        message = BatteryState()
        message.voltage = np.random.rand() * 10
        message.percentage = np.random.rand() * 10

        message.header.stamp = rospy.Time.now()

        message_publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    try:
        batteryIn_T_publisher()
    # capture the Interrupt signals
    except rospy.ROSInterruptException:
        pass
