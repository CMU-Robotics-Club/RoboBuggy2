#!/usr/bin/python3
# import rospy
# rospy.init_node("command", anonymous=False)


from buggy_steer import *
from buggy_brake import *
import numpy as np
import rospy
rospy.init_node("buggy_commands", anonymous=False)


def autonomy():
    for i in range(0, 100):
        set_steering_angle(i)
    
    for i in range(0,100):
        set_brake(np.random.rand()*100.0)

    rospy.spin()


if __name__ == '__main__':
    try:
        autonomy()

    except rospy.ROSInterruptException:
        pass
