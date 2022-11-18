#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

# TODO: write a test publisher node
# TODO: use https://wiki.ros.org/geonav_transform
# to convert gps coordinate to local xyz

class CoordinateLogger:
    def __init__(self, fileName):
        self.file = open(fileName, "x")
        self.isLogging = false

    def start(self):
        self.isLogging = true
        self.listener()

    def writeLine(self, data):
        for field in data:
            self.file.write(str(field))
            self.file.write(", ")
        self.file.write("\n")

    def navSatCb(self, data):
        if self.isLogging:
            self.writeLine(data)
        
    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("GPS", NavSatFix, navSatCb)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def stop(self):
        self.isLogging = false
        self.file.close()
        

if __name__ == '__main__':
    logger = CoordinateLogger("coordinate_list")
    logger.start()