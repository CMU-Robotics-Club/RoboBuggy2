#!/usr/bin/env python3

import rospy

# ROS Message Imports
from std_msgs.msg import Float32, Float64, Bool

def cb(msg):
    print(msg)


# For test topic remapping
# TODO: remove after rolls
if __name__ == "__main__":
    rospy.init_node("dummy")
    rospy.Subscriber("/input/steering", Float64, cb)
    while (not rospy.is_shutdown()):
        rospy.sleep(1)

