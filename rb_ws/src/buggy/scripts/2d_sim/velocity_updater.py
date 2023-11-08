#! /usr/bin/env python3
import rospy
from controller_2d import Controller
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import threading

class VelocityUpdater:
    RATE = 100

    def __init__(self):
        self.pose_subscriber = rospy.Subscriber(
            "sim_2d/utm", Pose, self.update_position
        )

        self.buggy_vel = 0.0
        self.time = 0.0

        self.controller = Controller()

        self.lock = threading.Lock()
    
    def update_position(self, new_pose: Pose):
        '''Callback function to update internal position variable'''
        with self.lock:
            self.position = new_pose.position

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
