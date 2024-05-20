#!/usr/bin/env python3
import argparse

import rospy
from std_msgs.msg import Float64
import numpy as np


"""
Debug Controller
Sends oscillating steering command for firmware and system level debug
"""
class DebugController():

    """
    @input: self_name, for namespace for current buggy
    Initializes steer publisher to publish steering angles
    Tick = 1ms
    """
    def __init__(self, self_name) -> None:
        self.steer_publisher = rospy.Publisher(
        self_name + "/buggy/input/steering", Float64, queue_size=1)
        self.rate = 1000

    # Outputs a continuous sine wave ranging from -50 to 50, with a period of 500 ticks
    def sin_steer(self, tick_count):
        return 50 * np.sin((2 * np.pi) * tick_count/500)

    #returns a constant steering angle of 42 degrees
    def constant_steer(self, _):
        return 42.0

    #Creates a loop based on tick counter
    def loop(self):
        rate = rospy.Rate(self.rate)
        tick_count = 0
        steer_cmd = 0

        #Currently uses sin_steer to send tick publishers
        while not rospy.is_shutdown():
            self.steer_publisher.publish(Float64(steer_cmd))

            tick_count += 1
            steer_cmd = self.sin_steer(tick_count)
            rate.sleep()


#Launch code for Rosnode, initializes node as debug_steer
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--self_name", type=str, help="name of ego-buggy", required=True)
    args, _ = parser.parse_known_args()

    rospy.init_node("debug_steer")
    d = DebugController(args.self_name)
    d.loop()
