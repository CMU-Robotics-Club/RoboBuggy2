# A script to log battery data to a .txt file

import csv
import rospy
from sensor_msgs.msg import BatteryState # Callback function to print the subscribed data on the terminal

# Should be culled or improved to actually log the battery levels, to a file, when ros bags should contain this if properly published??

file = open("battery_data.txt", "w", newline="")
writer = csv.writer(file)
writer.writerow(["Time", "Voltage", "Percentage"])


def callback_battery(battery_data):
    writer.writerow(
        [battery_data.header.stamp, battery_data.voltage, battery_data.percentage]
    )


def batteryIn_T_subscriber():
    rospy.init_node("batteryLogger", anonymous=False)
    rospy.Subscriber("BatteryIn_T", BatteryState, callback_battery)
    rospy.spin()


if __name__ == "__main__":
    try:
        batteryIn_T_subscriber()
    except rospy.ROSInterruptException:
        pass
