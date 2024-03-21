#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import numpy as np


class DebugController():

    """
    Debug Controller
    Sends oscillating steering command for firmware debug
    """

    def __init__(self) -> None:
        self.steer_publisher = rospy.Publisher(
        "SC/buggy/input/steering", Float64, queue_size=1)
        self.rate = 1000

    def sin_steer(self, tick_count):
        return 50 * np.sin((2 * np.pi) * tick_count/500)


    def constant_steer(self, tick_count):
        return 42.0



    def loop(self):
        rate = rospy.Rate(self.rate)
        tick_count = 0
        steer_cmd = 0

        while not rospy.is_shutdown():


            self.steer_publisher.publish(Float64(steer_cmd))


            tick_count += 1
            steer_cmd = self.sin_steer(tick_count)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("debug_steer")
    d = DebugController()
    d.loop()
