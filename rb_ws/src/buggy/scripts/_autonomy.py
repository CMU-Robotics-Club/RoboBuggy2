#!/usr/bin/python3

import numpy as np
import rospy
from buggy_commands import *

def main_autonomy():
    for i in range(0, 100000000):
        set_steer_angle(i)
        set_brake(-i)
        set_light("Running")
    rospy.spin()

try:
    main_autonomy()
#capture the Interrupt signals
except rospy.ROSInterruptException:
    pass
