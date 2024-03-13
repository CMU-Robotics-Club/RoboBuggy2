#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64


class DebugController():

    """
    Debug Controller
    Sends oscillating steering command for firmware debug
    """

    def __init__(self) -> None:
        self.steer_publisher = rospy.Publisher(
        "SC/buggy/input/steering", Float64, queue_size=1)
        self.rate = 100

    def loop(self):
        rate = rospy.Rate(self.rate)
        tick_count = 0
        steer_cmd = 0
        add = True

        while not rospy.is_shutdown():
            if (add):
                steer_cmd += 18 / 100
            else:
                steer_cmd -= 18 / 100

            self.steer_publisher.publish(Float64(steer_cmd))

            tick_count += 1
            if (tick_count == 200):
                tick_count = 0

            if (steer_cmd >= 18):
                add = False

            if (steer_cmd <= -18):
                add = True
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("debug_steer")
    d = DebugController()
    d.loop()
