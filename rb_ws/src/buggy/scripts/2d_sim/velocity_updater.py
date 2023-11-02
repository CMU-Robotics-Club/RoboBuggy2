#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import threading

class VelocityUpdater:
    RATE = 100

    def __init__(self):
        self.vel_publisher = rospy.Publisher(
            "buggy/velocity", Float64, queue_size=1
        )
        self.buggy_vel = 0.0
        
        self.rate = 100
        self.lock = threading.Lock()

    def tick(self):
        new_velocity = Float64()
        new_velocity.data = self.buggy_vel + accel / self.RATE

        self.buggy_vel = new_velocity.data
        self.vel_publisher.publish(new_velocity)
        
        self.time += 1 / self.RATE


if __name__ == "__main__":
    rospy.init_node("velocity_updater")
    vel = VelocityUpdater()
    rate = rospy.Rate(vel.RATE)

    while not rospy.is_shutdown():
        vel.tick()
