#! /usr/bin/env python3
import rospy
from controller_2d import Controller
from std_msgs.msg import Float64
import threading

class VelocityUpdater:
    RATE = 100

    def __init__(self):
        self.buggy_vel = 0.0
        self.time = 0.0

        self.lock = threading.Lock()

    def calculate_accel(self) -> float:
        if self.time < 20.0: return 1.0
        else: return -1.0

    def tick(self):
        accel = self.calculate_accel()

        new_velocity = self.buggy_vel + accel / self.RATE
        self.buggy_vel = new_velocity
        self.controller.set_velocity(new_velocity)

        self.time += 1 / self.RATE


if __name__ == "__main__":
    rospy.init_node("velocity_updater")
    vel = VelocityUpdater()
    rate = rospy.Rate(vel.RATE)

    while not rospy.is_shutdown():
        vel.tick()
        rate.sleep()
