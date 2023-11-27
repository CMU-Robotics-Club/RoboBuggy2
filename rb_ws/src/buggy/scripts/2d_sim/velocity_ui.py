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
        self.buggy_vel = 0 # So the buggy doesn't start moving without user input

        self.controller = Controller(buggy_name)
        self.lock = threading.Lock()

        self.root = tk.Tk()

        self.root.title('Manual Velocity')
        self.root.geometry('600x100')
        self.root.configure(background = '#342d66')

        self.scale = tk.Scale(self.root, from_=0, to=300, orient=tk.HORIZONTAL, length=500, width=30)
        self.scale.pack()

       

    def step(self):
        self.root.update_idletasks()
        self.root.update()
        '''Update velocity of buggy'''
        print("v: " + str(self.scale.get()))
        self.buggy_vel = self.scale.get()/10
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
