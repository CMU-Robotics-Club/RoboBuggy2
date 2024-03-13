#! /usr/bin/env python3
import sys
import threading
import tkinter as tk
from controller_2d import Controller
import rospy

class VelocityUI:
    def __init__(self, init_vel: float, buggy_name: str):
        self.buggy_vel = 0 # So the buggy doesn't start moving without user input
        self.controller = Controller(buggy_name)
        self.lock = threading.Lock()

        self.root = tk.Tk()

        self.root.title(buggy_name + ' Manual Velocity: scale = 0.1m/s')
        self.root.geometry('600x100')
        self.root.configure(background = '#342d66')

        self.scale = tk.Scale(self.root, from_=0, to=300, orient=tk.HORIZONTAL, length=500, width=30)
        self.scale.pack()

        self.root.bind("<Up>", lambda i: self.scale.set(self.scale.get() + 2))
        self.root.bind("<Down>", lambda d: self.scale.set(self.scale.get() - 2))

    def step(self):
        """sets velocity of buggy to the current scale value
            called once every tick
        """
        self.root.update_idletasks()
        self.root.update()
        '''Update velocity of buggy'''
        self.buggy_vel = self.scale.get()/10 # so we can set velocity with 0.1 precision
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