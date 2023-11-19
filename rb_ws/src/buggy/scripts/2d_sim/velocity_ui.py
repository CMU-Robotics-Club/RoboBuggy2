#! /usr/bin/env python3
import numpy as np
import rospy
import sys
from controller_2d import Controller
from std_msgs.msg import Float64
import threading
import math

class VelocityUI:
    
    def __init__(self, init_vel: float, buggy_name: str):
        # To provide the built in Foxglove UI a location to publish to
        self.manual_velocity_publisher = rospy.Publisher(buggy_name + "/input_velocity", Float64, queue_size=10)

        self.buggy_vel = 0 # So the buggy doesn't start moving without user input

        self.controller = Controller(buggy_name)
        self.lock = threading.Lock()

        
        self.manual_velocity_subscriber = rospy.Subscriber(
            buggy_name + "/input_velocity", Float64, self.update_velocity
        ) 



    def update_velocity(self, data: Float64):
        with self.lock:
            self.buggy_vel = data.data

    def step(self):
        '''Update velocity of buggy'''
        self.controller.set_velocity(self.buggy_vel)


if __name__ == "__main__":
    rospy.init_node("velocity_ui")

    init_vel = float(sys.argv[1])
    buggy_name = sys.argv[2]
    vel = VelocityUI(init_vel, buggy_name)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        vel.step()
        rate.sleep()
