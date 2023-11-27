#! /usr/bin/env python3
import numpy as np
import rospy
import sys
from controller_2d import Controller
from std_msgs.msg import Float64
import threading
import math
import tkinter as tk
import keyboard

class VelocityUI:
       

    def __init__(self, init_vel: float, buggy_name: str):
        # To provide the built in Foxglove UI a location to publish to
        self.manual_velocity_publisher = rospy.Publisher(buggy_name + "/velocity", Float64, queue_size=10)

        self.buggy_vel = 0 # So the buggy doesn't start moving without user input

        self.controller = Controller(buggy_name)
        self.lock = threading.Lock()

        root = tk.Tk()

        root.title('Manual Velocity')
        root.geometry('600x100')
        root.configure(background = '#342d66')

        self.scale = tk.Scale(root, from_=0, to=300, orient=tk.HORIZONTAL, length=500, width=30)
        self.scale.pack()

        root.mainloop()

    def step(self):
        '''Update velocity of buggy'''
        print("calling step!")
        print("v: " + self.scale.get())
        self.buggy_vel = self.scale.get()/10
        self.controller.set_velocity(self.buggy_vel())
        # if (keyboard.is_pressed('w')):
        #     self.controller.set_velocity(self.buggy_vel + 1)
        # elif (keyboard.is_pressed('s')):
        #     self.controller.set_velocity(self.buggy_vel - 1)
            


if __name__ == "__main__":
    rospy.init_node("velocity_ui")

    init_vel = float(sys.argv[1])
    buggy_name = sys.argv[2]
    vel = VelocityUI(init_vel, buggy_name)
    rate = rospy.Rate(100)
    print("im working")
    while not rospy.is_shutdown():
        print("stepping")
        vel.step()
        rate.sleep()
