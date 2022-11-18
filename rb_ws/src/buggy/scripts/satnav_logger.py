#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

# TODO: write a test publisher node
# TODO: use https://wiki.ros.org/geonav_transform
# to convert gps coordinate to local xyz

class CoordinateLogger:
    def __init__(self, fileName):
        self.file = open(fileName, "w")
        self.isLogging = False

    def start(self):
        self.isLogging = True
        self.listener()

    def writeLine(self, data):
        self.file.write(str(data))
        self.file.write("\n")

    def navSatCb(self, data):
        if self.isLogging:
            self.writeLine(data)
        
    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("GPS", NavSatFix, self.navSatCb)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def stop(self):
        self.isLogging = False
        self.file.close()
        

if __name__ == '__main__':
    logger = CoordinateLogger("coordinate_list")
    logger.start()